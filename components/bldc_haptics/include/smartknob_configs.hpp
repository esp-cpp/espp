static PB_SmartKnobConfig configs[] = {
    // int32_t position;
    // int32_t min_position;
    // int32_t max_position;
    // float position_width_radians;
    // float detent_strength_unit;
    // float endstop_strength_unit;
    // float snap_point;
    // char text[51];
    // pb_size_t detent_positions_count;
    // int32_t detent_positions[5];
    // float snap_point_bias;

    {
        0,
        0,
        -1, // max position < min position indicates no bounds
        10 * PI / 180,
        0,
        1,
        1.1,
        "Unbounded\nNo detents",
        0,
        {},
        0,
    },
    {
        0,
        0,
        10,
        10 * PI / 180,
        0,
        1,
        1.1,
        "Bounded 0-10\nNo detents",
        0,
        {},
        0,
    },
    {
        0,
        0,
        72,
        10 * PI / 180,
        0,
        1,
        1.1,
        "Multi-rev\nNo detents",
        0,
        {},
        0,
    },
    {
        0,
        0,
        1,
        60 * PI / 180,
        1,
        1,
        0.55, // Note the snap point is slightly past the midpoint (0.5); compare to normal detents
              // which use a snap point *past* the next value (i.e. > 1)
        "On/off\nStrong detent",
        0,
        {},
        0,
    },
    {
        0,
        0,
        0,
        60 * PI / 180,
        0.01,
        0.6,
        1.1,
        "Return-to-center",
        0,
        {},
        0,
    },
    {
        127,
        0,
        255,
        1 * PI / 180,
        0,
        1,
        1.1,
        "Fine values\nNo detents",
        0,
        {},
        0,
    },
    {
        127,
        0,
        255,
        1 * PI / 180,
        1,
        1,
        1.1,
        "Fine values\nWith detents",
        0,
        {},
        0,
    },
    {
        0,
        0,
        31,
        8.225806452 * PI / 180,
        2,
        1,
        1.1,
        "Coarse values\nStrong detents",
        0,
        {},
        0,
    },
    {
        0,
        0,
        31,
        8.225806452 * PI / 180,
        0.2,
        1,
        1.1,
        "Coarse values\nWeak detents",
        0,
        {},
        0,
    },
    {
        0,
        0,
        31,
        7 * PI / 180,
        2.5,
        1,
        0.7,
        "Magnetic detents",
        4,
        {2, 10, 21, 22},
        0,
    },
    // int32_t position;
    // int32_t min_position;
    // int32_t max_position;
    // float position_width_radians;
    // float detent_strength_unit;
    // float endstop_strength_unit;
    // float snap_point;
    // char text[51];
    // pb_size_t detent_positions_count;
    // int32_t detent_positions[5];
    // float snap_point_bias;
    {0, -6, 6, 60 * PI / 180, 1, 1, 0.55, "Return-to-center\nwith detents", 0, {}, 0.4},
};
