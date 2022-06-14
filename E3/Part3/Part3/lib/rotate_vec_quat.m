function rotated_vec = rotate_vec_quat(quat_rot, vec)
    quat_vec = [0; vec];
    rotate = quat_rot.*quat_vec.*quat_rot.^(-1);
    rotated_vec = [rotate(2), rotate(3), rotate(4)]';
end