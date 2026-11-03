"""Offline tests: replace SSH transport; never contact a board or store secrets."""
import contextlib
import getpass
import importlib.util
import io
import os
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


SOURCE = Path(__file__).resolve().parents[1] / 'ch341_driver_work/board_access.py'


class SSHException(Exception):
    pass


class AuthenticationException(SSHException):
    pass


class BadHostKeyException(SSHException):
    pass


class PasswordRequiredException(SSHException):
    pass


class RejectPolicy:
    pass


class BoardAccessTests(unittest.TestCase):
    def setUp(self):
        self.outcomes = []
        self.clients = []
        owner = self

        class FakeClient:
            def __init__(self):
                self.closed = False
                self.known_hosts_loaded = False
                self.policy = None
                self.attempts = []
                owner.clients.append(self)

            def load_system_host_keys(self):
                self.known_hosts_loaded = True

            def load_host_keys(self, path):
                self.known_hosts_loaded = True

            def set_missing_host_key_policy(self, policy):
                self.policy = policy

            def connect(self, hostname, **kwargs):
                self.attempts.append(dict(hostname=hostname, **kwargs))
                if owner.outcomes:
                    outcome = owner.outcomes.pop(0)
                    if outcome is not None:
                        raise outcome

            def close(self):
                self.closed = True

        paramiko = types.SimpleNamespace(
            SSHClient=FakeClient, SSHException=SSHException,
            AuthenticationException=AuthenticationException,
            PasswordRequiredException=PasswordRequiredException,
            BadHostKeyException=BadHostKeyException, RejectPolicy=RejectPolicy)
        spec = importlib.util.spec_from_file_location('board_access_test', SOURCE)
        self.module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {'paramiko': paramiko}):
            spec.loader.exec_module(self.module)
        clean_env = {key: value for key, value in os.environ.items() if not key.startswith('ROBOT_')}
        self.env = mock.patch.dict(os.environ, clean_env, clear=True)
        self.env.start()
        self.addCleanup(self.env.stop)

    def test_environment_password_replaces_personal_file(self):
        # A regression to the personal password file must not win over config.
        with mock.patch.dict(os.environ, {'ROBOT_PASSWORD': 'test-only-value'}):
            with mock.patch.object(Path, 'read_text', return_value='密码：old-local-file'):
                self.assertEqual(self.module.get_password(), 'test-only-value')

    def test_key_agent_success_never_requests_password(self):
        with mock.patch.dict(os.environ, {'ROBOT_HOST': 'board.example',
                                         'ROBOT_USER': 'robot', 'ROBOT_SSH_PORT': '2222',
                                         'ROBOT_SSH_KEY': '~/robot-test-key'}):
            with mock.patch.object(self.module, 'get_password', side_effect=AssertionError('unexpected password')):
                client = self.module.connect()
        attempt = client.attempts[0]
        self.assertEqual((attempt['hostname'], attempt['username'], attempt['port']),
                         ('board.example', 'robot', 2222))
        self.assertEqual(attempt['key_filename'], str(Path('~/robot-test-key').expanduser()))
        self.assertTrue(attempt['allow_agent'])
        self.assertTrue(attempt['look_for_keys'])
        self.assertNotIn('password', attempt)
        self.assertTrue(client.known_hosts_loaded)
        self.assertIsInstance(client.policy, RejectPolicy)
        self.assertFalse(client.closed)

    def test_environment_password_fallback_has_no_disk_or_console_output(self):
        self.outcomes = [AuthenticationException('key not accepted'), None]
        captured = io.StringIO()
        real_open, real_io_open = open, io.open

        def guard(original):
            def guarded(file, mode='r', *args, **kwargs):
                if any(flag in mode for flag in 'wax+'):
                    raise AssertionError('Credential flow must not write files')
                return original(file, mode, *args, **kwargs)
            return guarded

        with mock.patch.dict(os.environ, {'ROBOT_PASSWORD': 'test-only-value'}):
            with mock.patch('builtins.open', side_effect=guard(real_open)), \
                    mock.patch('io.open', side_effect=guard(real_io_open)), \
                    contextlib.redirect_stdout(captured), contextlib.redirect_stderr(captured):
                client = self.module.connect()
        self.assertEqual(len(self.clients), 2)
        self.assertTrue(self.clients[0].closed)
        self.assertEqual(client.attempts[0]['password'], 'test-only-value')
        self.assertFalse(client.attempts[0]['allow_agent'])
        self.assertFalse(client.attempts[0]['look_for_keys'])
        self.assertTrue(client.known_hosts_loaded)
        self.assertIsInstance(client.policy, RejectPolicy)
        self.assertEqual(captured.getvalue(), '')

    def test_interactive_password_fallback_without_available_keys(self):
        self.outcomes = [SSHException('No authentication methods available'), None]
        with mock.patch.object(sys.stdin, 'isatty', return_value=True), \
                mock.patch('getpass.getpass', return_value='interactive-test-value'):
            client = self.module.connect()
        self.assertEqual(client.attempts[0]['password'], 'interactive-test-value')
        self.assertEqual(client.attempts[0]['hostname'], '169.254.100.100')
        self.assertEqual(client.attempts[0]['username'], 'ubuntu')

    def test_noninteractive_authentication_error_is_clear(self):
        self.outcomes = [SSHException('No authentication methods available')]
        with mock.patch.object(sys.stdin, 'isatty', return_value=False), \
                mock.patch('getpass.getpass', side_effect=AssertionError('must not prompt')):
            with self.assertRaisesRegex(RuntimeError, 'ROBOT_PASSWORD'):
                self.module.connect()
        self.assertEqual(len(self.clients), 1)
        self.assertTrue(self.clients[0].closed)

    def test_host_key_errors_never_fall_back_to_password(self):
        for error in (BadHostKeyException('host key mismatch'),
                      SSHException('Server not found in known_hosts')):
            with self.subTest(error=type(error).__name__):
                self.clients.clear()
                self.outcomes = [error]
                with mock.patch.object(self.module, 'get_password', side_effect=AssertionError('must not prompt')):
                    with self.assertRaises(type(error)):
                        self.module.connect()
                self.assertEqual(len(self.clients), 1)
                self.assertTrue(self.clients[0].closed)
                self.assertIsInstance(self.clients[0].policy, RejectPolicy)

    def test_network_failure_does_not_become_password_retry(self):
        self.outcomes = [OSError('connection refused')]
        with mock.patch.object(self.module, 'get_password', side_effect=AssertionError('must not prompt')):
            with self.assertRaises(OSError):
                self.module.connect()
        self.assertEqual(len(self.clients), 1)
        self.assertTrue(self.clients[0].closed)

    def test_invalid_ports_are_rejected_before_creating_transport(self):
        for value in ('0', '-1', '65536', '22.5', 'ssh', ''):
            with self.subTest(port=value), mock.patch.dict(os.environ, {'ROBOT_SSH_PORT': value}):
                with self.assertRaisesRegex(ValueError, 'ROBOT_SSH_PORT'):
                    self.module.connect()
        self.assertEqual(self.clients, [])

    def test_port_boundaries_are_accepted(self):
        for value in ('1', '65535'):
            with self.subTest(port=value), mock.patch.dict(os.environ, {'ROBOT_SSH_PORT': value}):
                client = self.module.connect()
                self.assertEqual(client.attempts[0]['port'], int(value))

    def test_rejected_password_closes_transport(self):
        self.outcomes = [AuthenticationException('key rejected'), AuthenticationException('password rejected')]
        with mock.patch.dict(os.environ, {'ROBOT_PASSWORD': 'test-only-value'}):
            with self.assertRaises(AuthenticationException):
                self.module.connect()
        self.assertEqual(len(self.clients), 2)
        self.assertTrue(all(client.closed for client in self.clients))


if __name__ == '__main__':
    unittest.main(verbosity=2)
