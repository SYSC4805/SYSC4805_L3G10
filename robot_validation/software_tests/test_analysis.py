import importlib.util,math
from pathlib import Path
spec=importlib.util.spec_from_file_location('analysis',Path(__file__).resolve().parents[1]/'analyze_results.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
a=m.assess(dict(kind='speed',mode='CRUISE',reference_cm='200',reference_ms='10000',left_pulses='98',right_pulses='98'))
assert a['physical_average_cm_s']==20 and a['physical_average_within_target_2'] and a['each_encoder_within_10pct']
b=m.assess(dict(kind='speed',mode='CRUISE',reference_cm='200',reference_ms='10000',left_pulses='200',right_pulses='98'))
assert not b['each_encoder_within_10pct']
assert m.assess(dict(kind='turn',actual_degrees='94'))['angle_within_5_deg']
assert not m.assess(dict(kind='turn',actual_degrees='130'))['angle_within_5_deg']
assert m.assess(dict(kind='distance',mode='ROLL',reference_cm='100',left_pulses='49',right_pulses='49'))['each_wheel_within_5_cm']
for bad in ['0','-1','nan']:
 try:m.assess(dict(kind='speed',mode='SLOW',reference_cm='100',reference_ms=bad,left_pulses='49',right_pulses='49'))
 except ValueError:pass
 else:raise AssertionError('invalid time accepted')
print('PASS: analysis math, over-read detection, real turn-angle acceptance and invalid inputs')
