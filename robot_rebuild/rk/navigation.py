"""Waypoint upload/UI session only. Position and navigation run in the FPGA."""
from collections import deque
import json
import math
from pathlib import Path
from follow import RemoteFollowTrigger
from protocol import NAV_CLEAR, NAV_WRITE, NAV_COMMIT, NAV_POSE_RESET, navigation_packet


def navigation_failure_reason(telemetry, navigation):
    """Describe reported protection without guessing a failed physical sensor."""
    fault = telemetry.fault
    if fault:
        causes = []
        for mask, text in ((1, '控制通信超时'), (2, '姿态数据无效或跳变'),
                           (16, '停止许可断开'),
                           (32, '电机闭环或预测控制保护（起步反馈/编码器需核对）')):
            if fault & mask:
                causes.append(text)
        if fault & 0xFF00:
            causes.append('避障过程保护')
        return '、'.join(causes or ['控制保护']) + f'，故障0x{fault:04X}'
    if not telemetry.sensors_ready:
        missing = [text for mask, text in ((1, '控制通信'), (2, '姿态数据'), (16, '停止许可'))
                   if not telemetry.health & mask]
        return '启动条件失效：' + '、'.join(missing)
    return f'FPGA导航已中止（状态{navigation.state}，路径点{navigation.index+1}）'


def load_route(path):
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data,dict) or data.get('units')!='mm':
        raise ValueError('route requires units="mm" and points=[[x,y],...]')
    points=data.get('points')
    if not isinstance(points,list) or not 1<=len(points)<=8:
        raise ValueError('provide 1..8 waypoints')
    for i,point in enumerate(points):
        if not isinstance(point,list) or len(point)!=2:raise ValueError('each waypoint is [x,y]')
        navigation_packet(0,NAV_WRITE,index=i,x_mm=point[0],y_mm=point[1])
    return [tuple(p) for p in points]


class NavigationSession:
    POST_DISARM_S=.12
    START_TIMEOUT_S=4.
    REMOTE_START_TIMEOUT_S=.8
    REQUEST_TIMEOUT_S=12.
    def __init__(self, points, *, initialize_pose=True, remote_enabled=False):
        if not 1<=len(points)<=8:raise ValueError('1..8 points required')
        for i,(x,y) in enumerate(points):navigation_packet(0,NAV_WRITE,index=i,x_mm=x,y_mm=y)
        self.points=list(points)
        if not isinstance(initialize_pose,bool):raise ValueError('initialize_pose must be boolean')
        self.initialize_pose=initialize_pose
        if not isinstance(remote_enabled,bool):raise ValueError('remote_enabled must be boolean')
        # The existing FPGA key0 generation is contextual: person-follow in
        # follow mode, waypoint navigation only in this explicitly chosen mode.
        self.remote_trigger=RemoteFollowTrigger() if remote_enabled else None
        self.remote_start_pending=None
        self.remote_notice=''
        self.run_request=None
        self.enabled=False
        self.pending_disarm=True
        self.last_send=float('-inf')
        self.start_at=None
        self.start_sent_at=None
        self.start_deadline=None
        self.start_baseline=None
        self.stop_epoch=None
        self.grace_until=0.
        self.still_since=None
        self.waiting=None
        self.queue=deque()
        self.prepared=False
        self.route_confirmed=False
        self.config_started=False
        self.text='等待支持导航的FPGA固件'
        self.failure_reason=None
        self.upload()

    def stop(self, reason='已停车；E继续，R重设起点'):
        self.run_request=None
        self.remote_start_pending=None
        self.remote_notice=''
        self.enabled=False
        self.start_at=None
        self.start_sent_at=None
        self.start_deadline=None
        self.start_baseline=None
        self.pending_disarm=True
        self.waiting=None
        self.queue.clear()
        self.text=reason

    def upload(self):
        self.failure_reason=None
        self.stop('停车后上传路线；不会自动启动')
        self.prepared=False
        self.route_confirmed=False
        self.config_started=False
        self.queue.append((NAV_CLEAR,{}))
        for i,(x,y) in enumerate(self.points):
            self.queue.append((NAV_WRITE,dict(index=i,x_mm=x,y_mm=y)))
        self.queue.append((NAV_COMMIT,dict(count=len(self.points))))
        if self.initialize_pose:self.queue.append((NAV_POSE_RESET,{}))

    def reset_pose(self):
        self.failure_reason=None
        if not self.route_confirmed:
            self.upload()
            return
        self.stop('停车后重设当前位置和朝向为起点')
        self.prepared=False
        self.config_started=False
        self.queue.append((NAV_POSE_RESET,{}))

    def start(self, link, now):
        if self.enabled or self.start_at is not None:return False
        if isinstance(now,bool) or not isinstance(now,(int,float)) or not math.isfinite(now):return False
        t=link.telemetry if link.live(now) else None
        n=link.nav_telemetry if link.nav_live(now) else None
        if not self.prepared or not self.route_confirmed or not t or not n or not n.supported or not n.ready or not n.pose_ready or n.failed or n.done or t.fault or not t.sensors_ready:
            self.text=('暂不能启动：等待路线、定位及IMU/控制许可；恢复后重新按0' if self.remote_trigger else
                       '暂不能启动：等待上传、定位初始化及IMU/控制许可；必要时按R')
            return False
        if self.queue or self.waiting:return False
        if t.health&64 or t.command or t.state or t.velocity_l or t.velocity_r:
            self.text='暂不能启动：等待实际停稳及解除使能'
            return False
        if self.stop_epoch is not None and n.stop_epoch!=self.stop_epoch:
            self.stop_epoch=n.stop_epoch
            self.stop('遥控5停车事件，已取消导航启动；需重新明确启动')
            self.pending_disarm=False
            return False
        self.stop_epoch=n.stop_epoch
        self.failure_reason=None
        self.pending_disarm=True
        self.start_at=now+.25
        self.start_sent_at=None
        self.start_deadline=now+self.START_TIMEOUT_S
        self.start_baseline=(t.seq,n.seq)
        self.text='准备启动导航'
        return True

    def request_run(self,link,now):
        """Key7 authorizes one bounded prepare-and-run cycle, not future retries."""
        if isinstance(now,bool) or not isinstance(now,(int,float)) or not math.isfinite(now):
            self.stop('7键请求时钟无效，保持停车');return False
        t=link.telemetry if link.live(now) else None
        n=link.nav_telemetry if link.nav_live(now) else None
        if (t is None or n is None or not n.supported
                or t.health&0x12!=0x12 or t.fault):
            self.stop('7键请求未接受：检查遥测、IMU、停止许可及故障，再重新按7')
            return False
        if self.enabled or self.start_at is not None or self.run_request is not None:return True
        if not self.queue and not self.waiting:
            if not self.route_confirmed or not n.ready:self.upload()
            elif n.done or n.failed or not n.pose_ready:self.reset_pose()
        self.run_request=(now+self.REQUEST_TIMEOUT_S,now,n.stop_epoch,False,n.nav_mode_count)
        self.pending_disarm=True
        self.text='已收到7：先停车准备路线，确认后直接开始导航'
        return True

    def step(self, link, now, *, remote_allowed=True):
        try:
            self._advance_run_request(link,now,remote_allowed,allow_start=False)
            self._step(link,now,remote_allowed=remote_allowed)
            self._advance_run_request(link,now,remote_allowed)
        except (OSError,ValueError) as exc:
            if self.run_request is not None:
                self.stop('7键导航请求通信失败，已取消；恢复后重新按7')
            elif not self.enabled and self.start_at is None and not self.config_started and self.queue:
                self.pending_disarm=True
                self.text='等待串口连接；连接后只上传路线，不自动启动'
            else:self.stop('串口/配置错误，保持停车：'+str(exc))
        return (self.remote_notice+'；' if self.remote_notice else '')+self.text

    def _advance_run_request(self,link,now,allowed,*,allow_start=True):
        if self.run_request is None:return
        deadline,requested,epoch,seen,generation=self.run_request
        t=link.telemetry if link.live(now) else None
        n=link.nav_telemetry if link.nav_live(now) else None
        if not allowed or now>=deadline:
            self.stop('7键导航请求已取消/超时；准备好后重新按7');return
        if t is None or n is None:
            # A receive-only viewer upgrades its serial owner on this explicit
            # action. Allow its first snapshot, but never resume a lost link.
            if not seen and now-requested<.5:return
            self.stop('7键导航请求遥测中断；恢复后重新按7');return
        reset=(generation is not None and (n.nav_mode_count is None or n.nav_mode_count<generation))
        if reset or n.stop_epoch!=epoch or not n.supported or t.health&0x12!=0x12 or t.fault:
            self.stop('7键导航请求条件失效或已按5；重新按7才可启动')
            if n.stop_epoch!=epoch:self.pending_disarm=False
            return
        self.run_request=(deadline,requested,epoch,True,generation)
        if self.enabled:
            self.run_request=None
        elif (allow_start and self.prepared and not self.queue and not self.waiting and self.start_at is None
                and self._remote_stopped(t) and t.sensors_ready):
            self.start(link,now)

    def _step(self, link, now, *, remote_allowed=True):
        if isinstance(now,bool) or not isinstance(now,(int,float)) or not math.isfinite(now):
            raise ValueError('invalid monotonic clock')
        t=link.telemetry if link.live(now) else None
        n=link.nav_telemetry if link.nav_live(now) else None
        if n is not None and n.supported:
            if self.stop_epoch is not None and n.stop_epoch!=self.stop_epoch:
                self.stop_epoch=n.stop_epoch
                self.stop('遥控5停车事件；已取消导航并保留手动遥控准备')
                self.pending_disarm=False
                return
            self.stop_epoch=n.stop_epoch
        if self.remote_trigger is not None and self._remote_step(link,t,n,now,remote_allowed):
            return
        if self.pending_disarm:
            link.send()
            self.pending_disarm=False
            self.last_send=now
            if self.start_at is not None:self.start_sent_at=now
        if not t or not n or not n.supported:
            if self.enabled or self.start_at is not None or self.waiting:
                self.stop('遥测/导航数据超时，已停车；恢复后需明确重启')
            else:self.text='等待导航遥测；旧固件不能启动导航'
            return
        if self.enabled:
            if t.fault or not t.sensors_ready or n.failed:
                self.failure_reason=navigation_failure_reason(t,n) + (
                    '；已停车，检查后按0重新准备' if self.remote_trigger else '；已停车，检查后按R')
                self.stop(self.failure_reason)
            elif n.done:
                self.stop('所有路径点已完成；摆回起点后按0重新准备，再按0启动' if self.remote_trigger else '所有路径点已完成；R重新设定起点')
            elif now>self.grace_until and (not n.selected or not t.health&64):
                self.stop('遥控停车或控制权切换，导航已停止')
            elif now-self.last_send>=.1:
                link.send(True,1,8,nav=True)
                self.last_send=now
            if self.pending_disarm:
                link.send();self.pending_disarm=False;self.last_send=now
            if self.enabled:self.text=f'导航中：点{n.index+1}/{n.count}，状态{n.state}'
            return
        if self.start_at is not None:
            if (t.fault or not t.sensors_ready or n.failed or n.done or not n.ready or not n.pose_ready
                    or t.health&64 or t.command or t.state or t.velocity_l or t.velocity_r):
                self.stop('启动条件失效，保持停车')
            elif now>=self.start_deadline:
                self.stop('启动停车确认超时；需重新明确启动')
            elif now>=self.start_at and self._post_stop_feedback(link,t,n,now):
                link.send(True,1,8,nav=True)
                self.last_send=now;self.enabled=True;self.start_at=None;self.grace_until=now+.4
            else:self.text='等待停车后新的普通/导航遥测确认，不自动沿用旧快照'
            if self.pending_disarm:
                link.send();self.pending_disarm=False;self.last_send=now
            return
        # Configuring is always disarmed; DISARM heartbeats keep the command
        # link fresh without renewing a motion permission.
        if self.queue or self.waiting:
            if now-self.last_send>=.1:
                link.send();self.last_send=now
            if t.health&64 or t.velocity_l or t.velocity_r:
                self.still_since=None;self.text='等待车轮停止后配置路线';return
            if self.still_since is None:self.still_since=now
            if now-self.still_since<.25:return
            if self.waiting:
                seq,stamp,retries,baseline=self.waiting
                if n.seq!=baseline and n.ack_seq==seq and n.ack_status!=255:
                    if n.ack_status==0:
                        self.queue.popleft();self.waiting=None
                        if not self.queue:self.route_confirmed=True
                    elif n.ack_status==1:
                        if retries>=3:
                            self.stop('配置持续繁忙；停车后按0重新准备' if self.remote_trigger else '配置持续繁忙；停车后按L重试');return
                        op,fields=self.queue[0]
                        self.waiting=(link.configure_navigation(op,**fields),now,retries+1,n.seq)
                        return
                    else:self.stop(f'FPGA拒绝路线配置，状态{n.ack_status}');return
                elif now-stamp>1:
                    self.stop('路线配置无确认，保持停车；按0重新准备' if self.remote_trigger else '路线配置无确认，保持停车；按L重新上传');return
                else:return
            if self.queue:
                op,fields=self.queue[0]
                if op==NAV_POSE_RESET and (not t.sensors_ready or t.fault):
                    self.text='路线已传输；等待IMU/控制许可有效后重设起点，超声波缺测不阻止';return
                self.waiting=(link.configure_navigation(op,**fields),now,0,n.seq)
                self.config_started=True
                self.text='正在配置FPGA路线，等待逐条确认'
            else:
                # ACK snapshots can precede the core's pose/ready update.
                self.prepared=False
                self.text='等待FPGA确认路线和定位初始化'
        if self.route_confirmed and not self.queue and not self.waiting and n.ready and n.pose_ready and not n.failed and not n.done:
            self.prepared=True
            self.text=self.failure_reason or ('路线已准备：遥控0开始/继续，5停车' if self.remote_trigger else
                       '路线已准备：E启动，空格停止，R重设起点，L重传')
        elif not self.queue and not self.waiting and n.failed:
            self.prepared=False
            self.text=('FPGA导航已锁定失败；检查后按5停车，再按0重新准备' if self.remote_trigger else
                       'FPGA导航已锁定失败；检查传感器和校准，停车后按R')

    @staticmethod
    def _remote_stopped(t):
        # The UART freshness bit may expire while idle. Key0 may renew it via
        # DISARM, but must not queue a start while moving or sensors are faulty.
        return (t is not None and t.health&0x12==0x12 and not t.fault
                and not t.health&64 and not t.command and not t.state
                and not t.velocity_l and not t.velocity_r)

    def _remote_step(self,link,t,n,now,allowed):
        event=self.remote_trigger.observe(n,t is not None and n is not None and n.supported)
        if event=='stop':
            active=self.enabled or self.start_at is not None or self.remote_start_pending is not None
            if self.remote_trigger.remote_stop:
                self.stop('遥控5停车；准备好后重新点按0')
                self.pending_disarm=False  # Preserve FPGA manual IR readiness.
                return True
            if active:self.stop('遥控事件/连接中断，导航已取消；恢复后重新点按0')
        if not allowed:
            if self.remote_start_pending is not None:
                self.stop('导航启动已取消；返回导航画面后重新点按0')
            self.remote_start_pending=None
            return False  # Observe/consume hidden events; never replay them.
        if event=='start' and not self.enabled and self.start_at is None and self.remote_start_pending is None:
            if self.queue or self.waiting:
                self.remote_notice='0键未排队：等待路线准备完成后重新点按0'
            elif not self._remote_stopped(t):
                self.remote_notice='0键启动未接受：先按5停稳并检查IMU/故障，再重新点按0'
            elif not self.prepared or not self.route_confirmed or not n.ready or not n.pose_ready or n.done or n.failed:
                # Resetting a completed/failed route is preparation only. The
                # operator must click again after the new origin is confirmed.
                if not self.route_confirmed or not n.ready:self.upload()
                else:self.reset_pose()
                self.remote_notice='仅重新准备：以当前车位/车头为起点，准备完成后再按0启动'
            else:
                link.send()  # Release manual ownership without granting motion.
                self.pending_disarm=False
                self.last_send=now
                self.remote_start_pending=(now+self.REMOTE_START_TIMEOUT_S,now,t.seq,n.seq,n.stop_epoch)
                self.remote_notice='已收到0：等待停车后的新遥测，不沿用旧数据启动'
        pending=self.remote_start_pending
        if pending is not None:
            deadline,sent_at,seq,nav_seq,epoch=pending
            if now>=deadline or not self._remote_stopped(t) or n is None or n.stop_epoch!=epoch:
                self.stop('0键启动确认失败/超时；恢复后重新点按0')
            elif t.sensors_ready and self._feedback_since(link,t,n,now,sent_at,(seq,nav_seq)):
                self.remote_start_pending=None
                if self.start(link,now):self.remote_notice='0键已确认，准备启动导航'
                else:self.remote_notice='0键启动未接受；条件恢复后重新点按0'
        return False

    def _post_stop_feedback(self,link,t,n,now):
        if self.start_sent_at is None or self.start_baseline is None:return False
        return self._feedback_since(link,t,n,now,self.start_sent_at,self.start_baseline)

    @classmethod
    def _feedback_since(cls,link,t,n,now,sent_at,baseline):
        if t.seq==baseline[0] or n.seq==baseline[1]:return False
        for name in ('last_rx','last_nav_rx'):
            stamp=getattr(link,name,None)
            if (isinstance(stamp,bool) or not isinstance(stamp,(int,float)) or not math.isfinite(stamp)
                    or not sent_at+cls.POST_DISARM_S<=stamp<=now):return False
        return True
