while (1) {

    line_received = serial_read_line(serial_port, line, LINE_SIZE);

    if (line_received < 0) {
        break;
    }

    /*
     * Only use i, angle and distance when a complete valid line
     * was actually received.
     */
    if (line_received == 1) {

        if (extract_first_number(line, &i, &angle, &distance)) {

            /*
             * Make sure the LiDAR index is inside the array.
             */
            if (i >= 0 && i < LD06_SCAN_SIZE) {

                write_scan->d[i] = distance;

                /*
                 * Count this index only the first time it arrives
                 * during the current scan.
                 */
                if (scan_received[i] == 0) {
                    scan_received[i] = 1;
                    scan_points_received++;
                }

                /*
                 * All 503 measurements have now arrived.
                 */
                if (scan_points_received == LD06_SCAN_SIZE) {

                    scan_timestamp_us = get_time_us();
                    new_scan_ready = 1;

                    /*
                     * Clear the index tracker so the next scan
                     * can be collected.
                     */
                    for (int index = 0;
                         index < LD06_SCAN_SIZE;
                         index++) {

                        scan_received[index] = 0;
                    }

                    scan_points_received = 0;

                    printf(
                        "Complete LiDAR scan received at %" PRIu64 " us\n",
                        scan_timestamp_us
                    );
                }
            }
            else {
                printf("Invalid LiDAR index: %d\n", i);
            }
        }
        else {
            printf("Invalid line: %s\n", line);
        }
    }

    /*
     * ENCODER PLACEHOLDER
     *
     * Eventually your encoder serial extractor should do:
     *
     * write_scan->q1 = new_left_encoder_count;
     * write_scan->q2 = new_right_encoder_count;
     * encoder_timestamp_us = get_time_us();
     * new_encoders_ready = 1;
     */

    /*
     * Temporary behaviour while no encoder extractor exists:
     *
     * Treat each completed scan as also having a new encoder
     * measurement. q1 and q2 remain unchanged at zero.
     */
    if (new_scan_ready && !new_encoders_ready) {
        encoder_timestamp_us = scan_timestamp_us;
        new_encoders_ready = 1;
    }

    /*
     * Run SLAM only when both a complete scan and both new
     * encoder values are available.
     */
    if (new_scan_ready && new_encoders_ready) {

        /*
         * Use whichever sensor measurement arrived latest.
         */
        if (scan_timestamp_us > encoder_timestamp_us) {
            current_update_timestamp_us = scan_timestamp_us;
        }
        else {
            current_update_timestamp_us = encoder_timestamp_us;
        }

        /*
         * Calculate the time between complete combined sensor
         * updates.
         */
        if (previous_update_timestamp_us == 0) {
            elapsed_time_us = 0;
        }
        else {
            elapsed_time_us =
                current_update_timestamp_us
                - previous_update_timestamp_us;
        }

        previous_update_timestamp_us =
            current_update_timestamp_us;

        /*
         * CoreSLAM expects the absolute sensor timestamp.
         * It calculates the difference internally.
         */
        write_scan->timestamp =
            current_update_timestamp_us;

        printf(
            "Processing scan:\n"
            "  scan timestamp:    %" PRIu64 " us\n"
            "  encoder timestamp: %" PRIu64 " us\n"
            "  elapsed time:      %" PRIu64 " us\n"
            "  elapsed time:      %.6f seconds\n",
            scan_timestamp_us,
            encoder_timestamp_us,
            elapsed_time_us,
            elapsed_time_us / 1000000.0
        );

        ts_iterative_map_building(write_scan, &state);

        /*
         * Both measurements have now been consumed.
         * Wait for a new scan and new encoder readings.
         */
        new_scan_ready = 0;
        new_encoders_ready = 0;
    }
}