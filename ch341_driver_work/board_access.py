"""SSH access configured through ROBOT_* environment variables.

Install Paramiko separately. Trust the board's verified host key in the user's
~/.ssh/known_hosts before connecting; unknown or changed keys are rejected.
ROBOT_HOST / ROBOT_USER default to the original board address / ubuntu.
ROBOT_SSH_PORT (1..65535) and ROBOT_SSH_KEY are optional. Keys and the SSH agent
are tried first; login password fallback uses ROBOT_PASSWORD or a terminal
getpass prompt. Credentials are never saved by this module.
"""
import getpass
import os
from pathlib import Path
import sys

import paramiko


def get_password():
    """Read an optional process environment value or prompt without echo."""
    password = os.environ.get('ROBOT_PASSWORD')
    if password:
        return password
    if not sys.stdin.isatty():
        raise RuntimeError(
            'No usable SSH authentication in this non-interactive session. '
            'Configure an SSH key/agent or set ROBOT_PASSWORD for this process.')
    try:
        password = getpass.getpass('Robot login/sudo password: ')
    except EOFError:
        raise RuntimeError('Password input unavailable; configure an SSH key/agent or ROBOT_PASSWORD.') from None
    if not password:
        raise RuntimeError('No password supplied; configure an SSH key/agent or ROBOT_PASSWORD.')
    return password


def _client():
    client = paramiko.SSHClient()
    try:
        # Paramiko reads the user's known_hosts; a missing file does not grant trust.
        client.load_system_host_keys()
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
    except BaseException:
        client.close()
        raise
    return client


def connect():
    """Return an authenticated SSHClient; caller owns closing it."""
    try:
        port = int(os.environ.get('ROBOT_SSH_PORT', '22'))
    except ValueError:
        raise ValueError('ROBOT_SSH_PORT must be an integer from 1 to 65535.') from None
    if not 1 <= port <= 65535:
        raise ValueError('ROBOT_SSH_PORT must be an integer from 1 to 65535.')
    options = {
        'hostname': os.environ.get('ROBOT_HOST', '169.254.100.100'),
        'username': os.environ.get('ROBOT_USER', 'ubuntu'),
        'port': port, 'timeout': 8, 'auth_timeout': 8, 'banner_timeout': 8,
    }
    key = os.environ.get('ROBOT_SSH_KEY')
    key = str(Path(key).expanduser()) if key else None
    client = _client()
    try:
        client.connect(**options, key_filename=key, look_for_keys=True, allow_agent=True)
        return client
    except (paramiko.AuthenticationException, paramiko.PasswordRequiredException):
        client.close()
    except paramiko.SSHException as exc:
        client.close()
        # Paramiko uses this exact exception when no key or agent is available.
        # Host-key and transport errors must never enter password fallback.
        if str(exc) != 'No authentication methods available':
            raise
    except BaseException:
        client.close()
        raise

    password = get_password()
    client = _client()
    try:
        client.connect(**options, password=password, look_for_keys=False, allow_agent=False)
        return client
    except BaseException:
        client.close()
        raise

if __name__ == '__main__':
    client = connect()
    try:
        command = Path(sys.argv[1]).read_text(encoding='utf-8')
        _, stdout, stderr = client.exec_command('export LC_ALL=C\n' + command, timeout=60)
        print(stdout.read().decode('utf-8', errors='replace'))
        print(stderr.read().decode('utf-8', errors='replace'), file=sys.stderr)
        sys.exit(stdout.channel.recv_exit_status())
    finally:
        client.close()
