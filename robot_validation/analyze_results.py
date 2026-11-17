#!/usr/bin/env python3
"""Analyze independent measurements: python3 analyze_results.py measured_results.csv."""
import csv, math, sys
CM_PER_PULSE=math.pi*6.5/10

def number(row,key,positive=False):
    x=float(row[key])
    if not math.isfinite(x) or x<0 or (positive and x<=0):
        raise ValueError(f'{key} must be finite and '+('positive' if positive else 'nonnegative'))
    return x

def assess(row):
    kind=row['kind'].strip().lower()
    if kind=='turn':
        a=number(row,'actual_degrees')
        error=a-90
        return {'angle_error_deg':round(error,2),'angle_within_5_deg':abs(error)<=5}
    if kind not in ('distance','speed'):
        raise ValueError('kind must be distance, speed or turn')
    reference=number(row,'reference_cm',True)
    left=number(row,'left_pulses')*CM_PER_PULSE
    right=number(row,'right_pulses')*CM_PER_PULSE
    data={'left_cm':round(left,2),'right_cm':round(right,2)}
    if kind=='distance':
        data.update(left_error_cm=round(left-reference,2),right_error_cm=round(right-reference,2),
                    each_wheel_within_5_cm=abs(left-reference)<=5 and abs(right-reference)<=5)
        if row.get('mode','').strip().upper()=='DISTANCE':
            data['physical_stop_error_from_100cm']=round(reference-100,2)
            data['physical_stop_within_5cm']=abs(reference-100)<=5
        return data
    seconds=number(row,'reference_ms',True)/1000
    measured=reference/seconds
    lv,rv=left/seconds,right/seconds
    le,re=100*(lv-measured)/measured,100*(rv-measured)/measured
    data.update(physical_average_cm_s=round(measured,2),left_average_cm_s=round(lv,2),right_average_cm_s=round(rv,2),
                left_speed_error_pct=round(le,2),right_speed_error_pct=round(re,2),
                each_encoder_within_10pct=abs(le)<=10 and abs(re)<=10,
                physical_average_at_most_30=measured<=30)
    target={'CRUISE':20,'SLOW':10}.get(row.get('mode','').strip().upper())
    if target is not None:data.update(target_cm_s=target,physical_average_within_target_2=abs(measured-target)<=2)
    return data

def main():
    if len(sys.argv)!=2:raise SystemExit('Usage: python3 analyze_results.py measured_results.csv')
    count=0
    with open(sys.argv[1],newline='') as f:
        for line,row in enumerate(csv.DictReader(f),2):
            if not row.get('kind','').strip():continue
            count+=1
            try:print(f"{row.get('trial','?')} [{row['kind']}]: {assess(row)}")
            except (KeyError,ValueError,TypeError) as exc:print(f'INVALID row{line}: {exc}')
    if not count:print('No measured trials supplied. No pass results generated.')
    print('Speed averages do not prove peak speed. Results are per trial, not full-system acceptance.')
if __name__=='__main__':main()
