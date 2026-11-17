% demos/13_ros_middleware.m — ROS middleware: serialise/deserialise a Twist-like
% message to a byte vector and verify round-trip equality.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/13_ros_middleware.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

msg = struct(); msg.linear_x = 0.5; msg.linear_y = -0.2; msg.angular_z = 1.0;
fields = {"linear_x", "linear_y", "angular_z"};
buf = ros_serialise(msg, fields);
msg2 = ros_deserialise(buf, fields);

ok = isequal(msg.linear_x, msg2.linear_x) && ...
     isequal(msg.linear_y, msg2.linear_y) && ...
     isequal(msg.angular_z, msg2.angular_z);
fprintf("  serialised %d bytes\n", numel(buf));
fprintf("  round-trip: linear_x=%.3f (%.3f), linear_y=%.3f (%.3f), angular_z=%.3f (%.3f)\n", ...
    msg.linear_x, msg2.linear_x, msg.linear_y, msg2.linear_y, msg.angular_z, msg2.angular_z);
if ok, status = "MATCH"; else, status = "MISMATCH"; end
fprintf("  round-trip %s\n", status);

P = plot_helpers();
hf = P.newfig("13 ROS middleware — serialised byte layout");
P.plot_points(hf, [(1:numel(buf))', double(buf)], "-k.", "MarkerSize", 6);
xlabel("byte index"); ylabel("byte value");
title(sprintf("ROSB frame: %d bytes (magic + nfields + 3 x float64)", numel(buf)));

out = fullfile(repo_root, "output", "figures", "13_ros_middleware.png");
P.savefig(hf, out);
printf("13_ros_middleware: wrote %s  roundtrip=%s\n", out, status);
