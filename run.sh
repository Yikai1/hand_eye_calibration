python wql.py \
    --images "eye_hand_data/data2025072106/*.jpg" \
    --fx 1349.52898441919 --fy 1350.87190020274 --cx 971.157203214145 --cy 532.584981394128 \
    --k1 0.104989750334717 --k2 -0.164987200030023 --p1 0.0003 --p2 -0.0001 --k3 0.0 \
    --board_rows 5 --board_cols 5 --square_size 30 \
    # --visualize

python hand_eye_calibration.py \
    --images "eye_hand_data/data2025072106/*.jpg" \
    --poses   "./data/poses.yaml" \
    --rows 5 --cols 5 --square 0.03 \
    --intrinsics ./data/camera_intrinsics.yaml \
    --method TSAI \
    --out result_hand_eye.yaml