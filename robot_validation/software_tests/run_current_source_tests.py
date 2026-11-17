from pathlib import Path
import subprocess, tempfile, hashlib, sys, shutil
suite=Path(__file__).resolve().parents[1]
compiler=shutil.which("clang++") or shutil.which("g++")
if not compiler: raise SystemExit("Install clang++ or g++ first")
print("Current-source host tests. Mocked hardware, not physical acceptance.", flush=True)
with tempfile.TemporaryDirectory() as build:
 for name in ["Autonomous_SimHeading_Watchdog", "Distance_Speed_Validation", "Turn_Validation"]:
  sketch=suite/name/(name+".ino")
  text=sketch.read_text()
  print(name+" SHA256 "+hashlib.sha256(sketch.read_bytes()).hexdigest(), flush=True)
  anchor={"Autonomous_SimHeading_Watchdog":"void leftEncoderPulse(){", "Distance_Speed_Validation":"void leftPulse(){", "Turn_Validation":"void off(){"}[name]
  prototypes=(suite/"software_tests"/(name+"_prototypes.h")).read_text()
  text=text.replace(anchor,prototypes+"\n"+anchor,1)
  cpp=Path(build)/(name+".cpp")
  cpp.write_text((suite/"software_tests"/(name+"_harness.h")).read_text()+"\n"+text+"\n"+(suite/"software_tests"/(name+"_assertions.h")).read_text())
  exe=Path(build)/name
  subprocess.run([compiler,"-std=c++11","-Wno-format","-I",str(sketch.parent),str(cpp),"-o",str(exe)],check=True)
  subprocess.run([str(exe)],check=True,timeout=10)
 subprocess.run([sys.executable,str(suite/"software_tests/test_analysis.py")],check=True,timeout=10)
print("PASS: all current-source test suites", flush=True)
