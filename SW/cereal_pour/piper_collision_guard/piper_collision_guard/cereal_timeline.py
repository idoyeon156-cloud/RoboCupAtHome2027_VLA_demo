"""Pure v2 replay timing; object attachment switches only at waypoint boundaries."""
import numpy as np


class Timeline:
    def __init__(self,config):
        if config.get('version')!=2:raise ValueError('Expected a v2 planned sequence')
        self.config=config;self.segments=[]
        motion=config['settings']['motion']
        rates=np.r_[[np.deg2rad(float(motion['joint_speed_deg_s']))]*6,float(motion['opening_speed_mm_s'])/1000]
        if not np.isfinite(rates).all() or np.any(rates<=0):raise ValueError('Playback speeds must be positive')
        waypoints=config['waypoints']
        if len(waypoints)<2:raise ValueError('At least two waypoints required')
        def state(w):
            a=np.array([np.r_[np.deg2rad(w[s+'_deg']),w['opening_mm'][i]/1000] for i,s in enumerate(('left','right'))])
            if a.shape!=(2,7) or not np.isfinite(a).all():raise ValueError('Invalid waypoint')
            return a
        previous=waypoints[0];first=state(previous)
        hold=float(previous.get('hold_s',0))
        if hold<0 or not np.isfinite(hold):raise ValueError('Invalid hold')
        if hold:self.segments.append((first,first,hold,previous['name'],tuple(previous['held_objects'])))
        for current in waypoints[1:]:
            last=state(current);duration=float(np.max(np.abs(last-first)/rates))
            if duration>0:self.segments.append((first,last,duration,current['name'],tuple(previous['held_objects'])))
            hold=float(current.get('hold_s',0))
            if hold<0 or not np.isfinite(hold):raise ValueError('Invalid hold')
            if hold:self.segments.append((last,last,hold,current['name'],tuple(current['held_objects'])))
            first=last;previous=current
        self.duration=sum(x[2] for x in self.segments)
        if self.duration<=0:raise ValueError('Empty playback')
        # No discontinuous wrap from the last pose back to the first is permitted.
        if not np.allclose(state(waypoints[0]),state(waypoints[-1]),atol=1e-10):raise ValueError('Loop must return to its initial state')

    def sample(self,seconds):
        elapsed=float(seconds)%self.duration
        for first,last,duration,name,held in self.segments:
            if elapsed<duration:return first+(last-first)*(elapsed/duration),name,held
            elapsed-=duration
        return self.segments[0][0],self.segments[0][3],self.segments[0][4]
