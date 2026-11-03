"""Visual target following; selection never grants motion permission."""
from dataclasses import dataclass
import math


class RemoteFollowTrigger:
    """Consume fresh FPGA key generations; never replay a baseline/reconnect."""
    def __init__(self, count_field='follow_count'):
        if count_field not in ('follow_count','nav_mode_count'):
            raise ValueError('unknown remote event counter')
        self.count_field=count_field
        self.previous = None
        self.remote_stop = False

    def observe(self, nav, live):
        self.remote_stop = False
        count=getattr(nav,self.count_field,None)
        if not live or isinstance(count,bool) or not isinstance(count,int) or not 0<=count<=65535:
            had_link = self.previous is not None
            self.previous = None
            return 'stop' if had_link else None
        current = (count, nav.stop_epoch)
        previous, self.previous = self.previous, current
        if previous is None:
            return None
        count, stop = current
        old_count, old_stop = previous
        if stop != old_stop:
            self.remote_stop = True
            return 'stop'  # FPGA already stopped and granted manual IR control.
        if count < old_count or count > old_count + 1:
            return 'stop'  # STOP wins coalesced events, reset and missed bursts.
        return 'start' if count == old_count + 1 else None


@dataclass(frozen=True)
class FollowCommand:
    armed: bool = False
    command: int = 0
    speed: int = 0
    reason: str = 'select target, then press remote 0'


class TargetFollower:
    MAX_AGE = .4
    STOP_MM = 600
    RESUME_MM = 600
    MIN_SPEED = 4
    MAX_SPEED = 12

    def __init__(self, speed=8, *, category=0):
        self.set_speed(speed)
        if isinstance(category,bool) or not isinstance(category,int) or category not in (0,39):
            raise ValueError('only person(0) or bottle(39) targets are supported')
        self.category = category
        self.label='person' if category==0 else 'bottle'
        self.box = None
        self.selected_at = float('-inf')
        self.enabled = False
        self.hold_distance = False
        self.reason = f'waiting for automatic {self.label} selection; explicit start required'

    def set_speed(self, speed):
        """Change the preset only; never select, ARM or release a distance hold."""
        if (isinstance(speed,bool) or not isinstance(speed,int)
                or not self.MIN_SPEED <= speed <= self.MAX_SPEED):
            raise ValueError('speed 4..12 required')
        self.speed = speed

    def select_best(self, objects, captured, now):
        """Select the highest-confidence configured category; never re-arm."""
        if self.enabled:
            return False
        self.stop(f'no valid {self.label} detected', clear=True)
        if not math.isfinite(captured) or not 0 <= now-captured <= self.MAX_AGE:
            return False
        candidates = [(s, tuple(b)) for c,s,b in objects
                      if c == self.category and math.isfinite(s) and s >= .45
                      and self.valid_box(b)]
        if not candidates:
            return False
        self.box = max(candidates, key=lambda item: item[0])[1]
        self.selected_at = captured
        self.reason = f'highest-confidence {self.label} selected; explicit start required'
        return True

    def stop(self, reason='stopped; press remote 0 to restart', clear=False):
        self.enabled = False
        self.reason = reason
        if clear:
            self.box = None

    @staticmethod
    def valid_box(box):
        return len(box)==4 and all(math.isfinite(v) for v in box) and box[2]>box[0] and box[3]>box[1]

    def select(self, objects, x, y, captured, now):
        self.stop('no target at click', clear=True)
        if not math.isfinite(captured) or not 0 <= now-captured <= self.MAX_AGE:return False
        candidates=[b for c,s,b in objects if c==self.category and math.isfinite(s) and s>=.45
                    and self.valid_box(b) and b[0]<=x<=b[2] and b[1]<=y<=b[3]]
        if not candidates:return False
        self.box=tuple(min(candidates,key=lambda b:(b[2]-b[0])*(b[3]-b[1])))
        self.selected_at=captured
        self.reason='target selected; press remote 0'
        return True

    def start(self, now):
        if self.box is None or not 0 <= now-self.selected_at <= self.MAX_AGE:
            self.stop(f'selection expired; wait for a fresh {self.label} detection',clear=True)
            return False
        self.enabled=True
        self.hold_distance=False
        return True

    def track(self, objects, captured, now):
        if self.box is None:return False
        if not math.isfinite(captured) or not 0<=now-captured<=self.MAX_AGE:
            self.stop('camera stale; select target again',clear=True);return False
        if captured==self.selected_at:return True
        if captured<self.selected_at:
            self.stop('out-of-order image; select target again',clear=True);return False
        a=self.box
        area=(a[2]-a[0])*(a[3]-a[1])
        candidates=[]
        for category,score,b in objects:
            if category!=self.category or not math.isfinite(score) or score<.45 or not self.valid_box(b):continue
            other=(b[2]-b[0])*(b[3]-b[1])
            intersection=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
            overlap=intersection/(area+other-intersection)
            if overlap>=.3 and .5<=other/area<=2:candidates.append((overlap,tuple(b)))
        candidates.sort(reverse=True)
        # Ambiguous overlap can switch identity during crossings: stop instead.
        if not candidates or (len(candidates)>1 and candidates[0][0]-candidates[1][0]<.15):
            self.stop('target lost/ambiguous; press remote 0 to restart',clear=True);return False
        self.box=candidates[0][1]
        self.selected_at=captured
        return True

    def update(self, objects, captured, now, width, telemetry, live):
        tracked=self.track(objects,captured,now)
        if not self.enabled:return FollowCommand(reason=self.reason)
        if not live or telemetry is None or telemetry.fault or not telemetry.sensors_ready or telemetry.state!=0:
            self.stop('telemetry/sensor/control fault; press remote 0 after recovery')
            return FollowCommand(reason=self.reason)
        if not tracked or not math.isfinite(width) or width<=0:
            self.stop('invalid image; select target again',clear=True)
            return FollowCommand(reason=self.reason)
        # Only real, valid observations can request distance holding. Missing
        # sonar releases this hold; it neither invents a measured FAR nor faults.
        distances=[d for bit,d in ((4,telemetry.distance_l),(8,telemetry.distance_r))
                   if telemetry.health & bit and 0<d<65535]
        if not distances:self.hold_distance=False
        elif min(distances)<=self.STOP_MM:self.hold_distance=True
        elif min(distances)>=self.RESUME_MM:self.hold_distance=False
        if self.hold_distance:return FollowCommand(True,0,0,'distance hold (<=60cm)')
        center=(self.box[0]+self.box[2])/2/width
        # Large nearby image also stops: ultrasound is not target-specific depth.
        if (self.box[2]-self.box[0])/width>=.65:
            return FollowCommand(True,0,0,'target fills image; hold')
        command=3 if center<.42 else 4 if center>.58 else 1
        return FollowCommand(True,command,self.speed,'following selected target')
