# Subsystem 3 — test log

**Subsystem:** Consumer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

**Steps for every test**

1. Robot power **off**.
2. Hold BOOTSEL, plug in USB, drag `ir_sensor_check.uf2` onto **RPI-RP2**.
3. VS Code → Serial Monitor → pick the Pico's COM port → Start Monitoring.
4. Hold the car at its normal driving height over the track paper and move it by hand.
5. Copy the Serial Monitor output for the test into the log file named in the Results table.

Each output line should look like this:
`BC:<white/BLACK> changes <n> | MID:<white/BLACK> | EDGE:<0..4095> [...] min <n> max <n> zone <WHITE/EDGE/BLACK> ...`

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-M01 | Middle sensor detects the line | Middle sensor, GP28, digital | manual | Move the middle sensor from plain white onto the black line and back, 3 times | MID:white over white and MID:BLACK over the line, every time |
| S3-M02 | Barcode sensor detects black and white | Left sensor, GP26, digital | manual | Move the left sensor from plain white onto a black bar and back, 3 times | BC:white over white and BC:BLACK over black, every time. changes goes up by 2 for each on-and-off |
| S3-M03 | Edge sensor reading on white | Right sensor, GP27, analogue, white limit EDGE_WHITE_MAX_ADC = 185 | manual | Hold the right sensor over plain white for 5s, note the lowest and highest EDGE value | EDGE stays below 185: zone WHITE |
| S3-M04 | Edge sensor reading half on the line's edge | Right sensor, GP27, analogue, edge band 185–400 (EDGE_WHITE_MAX_ADC, EDGE_ON_EDGE_MAX_ADC) | manual | Hold the right sensor half over the line's right edge for 5s, note the lowest and highest EDGE value | EDGE should stay between 185 and 400: zone EDGE |
| S3-M05 | Edge sensor reading on black | Right sensor, GP27, analogue, black above EDGE_ON_EDGE_MAX_ADC = 400 | manual | Hold the right sensor fully over the black line for 5 s, note the lowest and highest EDGE value | EDGE stays above 400: zone BLACK |
| S3-M06 | Barcode sensor catches every bar of a full barcode | Left sensor, GP26, one Code 39 barcode = 29 bars and gaps = 30 changes | manual | Note changes. Slide one barcode under the left sensor at a steady pace, start to end, touching no other black marks. Note changes again. Repeat 3 times | changes goes up by exactly **30** each time (fewer = thin bars were missed) |
| S3-M07 | A real barcode is decoded to the right letter | Left sensor, GP26, barcode_decode_code39() in line_barcode.c, Serial Monitor line: [BARCODE] read '<letter>' … | manual | Flash app.uf2 instead of ir_sensor_check.uf2, open the Serial Monitor. Pass the car over one printed barcode so the left sensor crosses it from start to end. Take note of the letter printed. | Serial Monitor prints [BARCODE] read '<letter>' with the letter printed under the barcode. Also note the distance shown (`over N mm`, expected about 141 mm) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console,
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-09 | S3-M01 | `no commit` | `Ozzy/Vincent` | FAIL | Serial Monitor, read by eye (no log saved) | Sensor switched black/white correctly, but its state appeared under `BC:` instead of `MID:`. Cause: Wrote middle and barcode sensors as being attached to the wrong pins. Fix: pin numbers swapped in code (`IR_MIDDLE_D0_PIN` = 28, `IR_BARCODE_D0_PIN` = 26). Re-test needed. |
| 2026-11-09 | S3-M02 | `no commit` | `Ozzy/Vincent` | FAIL | Serial Monitor, read by eye (no log saved) | Sensor switched black/white correctly, but its state appeared under `MID:` instead of `BC:`. Wrote middle and barcode sensors as being attached to the wrong pins. Fix: pin numbers swapped in code (`IR_MIDDLE_D0_PIN` = 28, `IR_BARCODE_D0_PIN` = 26). Re-test needed. |
| 2026-11-09 | S3-M03 | `no commit` | `Ozzy/Vincent` | PASS | Serial Monitor, read by eye (no log saved) | First measurement: EDGE 120–160 on white (below 185). |
| 2026-11-09 | S3-M04 | `no commit` | `Ozzy/Vincent` | FAIL | Serial Monitor, read by eye (no log saved) | First measurement: EDGE 140–170 half on the edge, overlapping with the white range. Adjusted height, no real improvement. Covering the sensor from ambient light tried and didn't really improve readings but might test further. Sensor's potentiometer readjusted and tested (See below). |
| 2026-11-09 | S3-M05 | `no commit` | `Ozzy/Vincent` | PASS | Serial Monitor, read by eye (no log saved) | First measurement: EDGE 1500–2000 on black (above 400). |
| 2026-11-09 | S3-M03 | `no commit` | `Ozzy/Vincent` | PASS | Serial Monitor, read by eye (no log saved) | After readjusting: EDGE 150–180 on white (below 185). |
| 2026-11-09 | S3-M04 | `no commit` | `Ozzy/Vincent` | PASS | Serial Monitor, read by eye (no log saved) | After readjusting: EDGE 190–230 half on the edge (within 185–400). Gap to white is small (~10); thresholds 185/400 set from these readings. Reading jumps sharply between white, edge, and black, so code will use three zones instead of a smooth scale. |
| 2026-11-09 | S3-M05 | `no commit` | `Ozzy/Vincent` | PASS | Serial Monitor, read by eye (no log saved) | After readjusting: EDGE 2500–3000 on black (above 400). |
| 2026-11-12 | S3-M07 | `no commit` | `Vincent` | PASS | Serial Monitor, read by eye (no log saved) | Barcode `*X*` read correctly: Serial Monitor showed `[BARCODE] read 'X' …`. 'X' is not in the A–D command table, so it is decoded but triggers no turn. |
