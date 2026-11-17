% demos/14_hardware_connectivity.m — Hardware connectivity: build a framed
% serial packet, parse+verify it, and confirm a corrupted packet is rejected.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/14_hardware_connectivity.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

[pkt, len] = serial_packet(0x01, uint8([10, 20, 30, 40]));
fprintf("  packet: %d bytes = ", len+3);
disp(num2str(pkt'));

[cmd, payload, ok] = serial_parse(pkt);
fprintf("  parse ok=%d  cmd=0x%02x  payload=%s\n", ok, cmd, ...
    num2str(reshape(payload, 1, [])));

% corrupted packet: flip a payload byte -> CRC must fail
pkt_bad = pkt; pkt_bad(6) = bitxor(pkt_bad(6), 0xFF);
[cmd_b, payload_b, ok_b] = serial_parse(pkt_bad);
fprintf("  corrupted parse ok=%d (expected 0)\n", ok_b);

P = plot_helpers();
hf = P.newfig("14 hardware — serial packet bytes");
P.plot_points(hf, [(1:numel(pkt))', double(pkt)], "-k.", "MarkerSize", 6);
xlabel("byte index"); ylabel("byte value");
title("frame: [0xAA][len][cmd][payload...][crc8]");

out = fullfile(repo_root, "output", "figures", "14_hardware_connectivity.png");
P.savefig(hf, out);
printf("14_hardware_connectivity: wrote %s\n", out);
