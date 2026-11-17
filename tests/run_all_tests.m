% tests/run_all_tests.m — Run every test file in a fresh headless octave-cli
% process; exit code 0 iff every test file exits 0.
%
% Run: cd <repo>; octave-cli --no-gui --quiet --eval "run('tests/run_all_tests.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));  % tests/ -> repo root
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
testfiles = {"tests/test_core.m", "tests/test_batchA.m", ...
             "tests/test_batchB.m", "tests/test_batchC.m"};
allpass = true;
for i = 1:numel(testfiles)
  sp = strrep(fullfile(repo_root, "setup_paths.m"), "\\", "/");
  tf = strrep(fullfile(repo_root, testfiles{i}), "\\", "/");
  cmd = sprintf('%s --no-gui --quiet --eval "run(%s); run(%s);"', ...
      octave_bin, sprintf("'%s'", sp), sprintf("'%s'", tf));
  [status, ~] = system(cmd);
  if status == 0
    printf("PASS: %s\n", testfiles{i});
  else
    printf("FAIL: %s  (octave exit status %d)\n", testfiles{i}, status);
    allpass = false;
  end
end
if allpass
  printf("=== ALL TESTS PASS ===\n");
  exit(0);
else
  printf("=== SOME TESTS FAILED ===\n");
  exit(1);
endif
