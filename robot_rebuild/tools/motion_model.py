"""Offline identification of the existing wheel-PI response, never motor control.

The recorded target belongs to the interval ending at timestamp_ms. The measured
velocity is that interval's encoder count. Fits are approximate discrete models,
not a substitute for powered validation or an IMU angle-scale measurement.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def ready(row):
    return int(row['flags']) & 0x1b == 0x0b and not int(row['flags']) & 512


def fit_model(rows, yaw_sign=1):
    if yaw_sign not in (-1,1):raise ValueError('yaw_sign must be +1 or -1')
    if len(rows)<61:raise ValueError('need at least 61 consecutive 20ms samples')
    pairs=[]
    for a,b in zip(rows,rows[1:]):
        if ((int(b['timestamp_ms'])-int(a['timestamp_ms'])) & 0xffffffff)!=20:continue
        if not ready(a) or not ready(b):continue
        if any(abs(int(b[k]))>512 for k in ('velocity_l','velocity_r')):continue
        if any(abs(int(b[k]))>80 for k in ('target_l','target_r')):continue
        pairs.append((a,b))
    if len(pairs)<60:raise ValueError('insufficient healthy, unbraked, consecutive 50Hz data')
    result={'samples':len(rows),'usable_intervals':len(pairs),'sample_ms':20,'wheels':{}}
    for side in ('l','r'):
        train=pairs[:len(pairs)*4//5];test=pairs[len(pairs)*4//5:]
        def residual(pair,shift):
            a,b=pair;v=int(a['velocity_'+side]);u=int(b['target_'+side])
            return int(b['velocity_'+side])-(v+(u-v)/(1<<shift))
        excitation=sum((int(b['target_'+side])-int(a['velocity_'+side]))**2 for a,b in train)
        if excitation<100:raise ValueError(f'{side} wheel lacks changing speed input; no response parameter can be fitted')
        shift=min(range(6),key=lambda n:sum(residual(pair,n)**2 for pair in train))
        result['wheels'][side]={'alpha_shift':shift,'alpha':1/(1<<shift),
            'train_rmse_counts':math.sqrt(statistics.mean(residual(p,shift)**2 for p in train)),
            'validation_rmse_counts':math.sqrt(statistics.mean(residual(p,shift)**2 for p in test))}
    # Aggregate 200ms of encoder differences to avoid comparing an asynchronous
    # 45Hz IMU frame against just one 20ms wheel interval.
    rotations=[]
    for start in range(0,len(rows)-10,10):
        window=rows[start:start+11]
        if not all(ready(r) for r in window):continue
        if any(((int(b['timestamp_ms'])-int(a['timestamp_ms'])) & 0xffffffff)!=20 for a,b in zip(window,window[1:])):continue
        diff=sum(int(r['velocity_r'])-int(r['velocity_l']) for r in window[1:])
        angle=int(window[-1]['imu_heading'])-int(window[0]['imu_heading'])
        angle=(angle+18000)%36000-18000
        if abs(diff)>=8 and abs(angle)>=200:rotations.append((diff,angle*yaw_sign))
    gain=None
    if len(rotations)>=8:
        gain=sum(x*y for x,y in rotations)/sum(x*x for x,y in rotations)
        if not 0<gain<=16:raise ValueError('turn response inconsistent: verify IMU/encoder signs and units')
    result['turn_windows']=len(rotations)
    result['yaw_gain_q8']=round(gain*256) if gain is not None else 0
    result['parameters']={'PRED_ALPHA_L_SHIFT':result['wheels']['l']['alpha_shift'],
        'PRED_ALPHA_R_SHIFT':result['wheels']['r']['alpha_shift'],
        'PRED_YAW_GAIN_Q8':result['yaw_gain_q8'],'PRED_MODEL_VALIDATED':0}
    result['validation_required']=True
    result['note']='Offline fit only. Keep MODEL_VALIDATED=0 until independent low-speed powered tests pass. Zero yaw gain means insufficient turn data.'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',type=Path)
    parser.add_argument('--yaw-sign',type=int,choices=(-1,1),default=1)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    with args.csv.open(encoding='utf-8-sig',newline='') as f:
        rows=list(csv.DictReader(f))
    try:result=fit_model(rows,args.yaw_sign)
    except (ValueError,KeyError) as exc:parser.error(str(exc))
    text=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
    if args.output:args.output.write_text(text,encoding='utf-8')
    print(text,end='')


if __name__=='__main__':main()
