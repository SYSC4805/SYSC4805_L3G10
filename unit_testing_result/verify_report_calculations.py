from pathlib import Path
from datetime import datetime,date
import json,math
rows=json.loads((Path(__file__).parent/"baseline_activities.json").read_text())
assert sum(int(r["hours"]) for r in rows)==240
print("Source: report Table 3.1. Original baseline, not actual hours.")
for cutoff in [date(2026,11,10),date(2026,11,16),date(2026,11,19),date(2026,12,10)]:
 total=500 if cutoff>=date(2026,10,22) else 0
 print("\nAs of",cutoff)
 for r in rows:
  start,end=[datetime.strptime(t+" 2026","%b %d %Y").date() for t in r["window"].split(" to ")]
  days=(end-start).days+1
  assert days==int(r["days"])
  fraction=min(1,max(0,((cutoff-start).days+1)/days))
  value=int(r["hours"])*50*fraction;total+=value
  if cutoff==date(2026,11,16):print(r["id"],"fraction",round(fraction,6),"PV CAD",round(value,2))
 print("TOTAL CAD",round(total,2),"percent",round(total/12500*100,2))
 expected={date(2026,11,10):6038.10,date(2026,11,16):7764.29,date(2026,11,19):8833.33,date(2026,12,10):12500.00}
 assert abs(total-expected[cutoff])<0.005
print("\nLine margins from observed extrema in Table 6.5")
for side,floor_max,tape_min,black,clear in [("left",3105,3951,3500,3300),("right",2840,3906,3400,3100)]:
 print(side,"tape margin",tape_min-black,"clear margin",clear-floor_max)
 assert tape_min>black and floor_max<clear
print("Wheel circumference cm",round(math.pi*6.5,6))
print("Left manual pulses/rev",sum([100,100,100])/30)
print("Right manual pulses/rev",sum([100,101,103])/30)
print("PASS: report calculation checks. No physical accuracy inferred.")
