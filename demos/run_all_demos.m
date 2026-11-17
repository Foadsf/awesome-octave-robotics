% demos/run_all_demos.m — Run every demo in a fresh headless octave-cli process
% and confirm each writes its figure PNG(s). Exit code 0 iff all demos pass.
%
% Run: cd <repo>; octave-cli --no-gui --quiet --eval "run('demos/run_all_demos.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));  % demos/ -> repo root
run(fullfile(repo_root, "setup_paths.m"));

if ispc()
  cand = fullfile (OCTAVE_HOME (), "bin", "octave-cli.exe");
  if (exist (cand, "file"))
    octave_bin = sprintf ('"%s"', cand);
  else
    octave_bin = "octave-cli";
  endif
else
  cand = fullfile (OCTAVE_HOME (), "bin", "octave-cli");
  if (exist (cand, "file"))
    octave_bin = sprintf ('"%s"', cand);
  else
    octave_bin = "octave-cli";
  endif
endif
demos = {
  "01_ground_vehicle.m",   "01_ground_vehicle.png",
  "02_manipulation.m",     "02_manipulation.png",
  "03_legged_locomotion.m","03_legged_locomotion.png",
  "04_robot_modeling.m",   "04_robot_modeling.png",
  "05_perception.m",       "05_perception.png",
  "06_mapping_slam.m",     "06_mapping_slam.png",
  "07_motion_planning.m",  "07_motion_planning.png",
  "08_motion_control.m",   "08_motion_control.png",
  "09_uav.m",              "09_uav.png",
  "10_marine_auv.m",       "10_marine_auv.png",
  "11_automated_driving.m","11_automated_driving.png",
  "12_simulators.m",       "12_simulators.png",
  "13_ros_middleware.m",   "13_ros_middleware.png",
  "14_hardware_connectivity.m","14_hardware_connectivity.png"
};
figdir = fullfile(repo_root, "output", "figures");
allpass = true;
nd = size(demos, 1);
for i = 1:nd
  demo = demos{i, 1}; png = demos{i, 2};
  tf = strrep(fullfile(repo_root, "demos", demo), "\\", "/");
  cmd = sprintf('%s --no-gui --quiet --eval "run(%s);"', ...
      octave_bin, sprintf("'%s'", tf));
  [status, ~] = system(cmd);
  png_ok = exist(fullfile(figdir, png), "file") == 2;
  if status == 0 && png_ok
    printf("PASS: %s  -> %s\n", demo, png);
  else
  if png_ok, pngst = "ok"; else, pngst = "missing"; end
  printf("FAIL: %s  (octave status %d, png %s)\n", demo, status, pngst);
    allpass = false;
  end
end
if allpass
  printf("=== ALL DEMOS PASS ===\n");
  exit(0);
else
  printf("=== SOME DEMOS FAILED ===\n");
  exit(1);
endif
