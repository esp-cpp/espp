#include "esp_lcd_ili9881c.h"

[[maybe_unused]] static const ili9881c_lcd_init_cmd_t
    tab5_lcd_ili9881c_specific_init_code_default[] = {
        // {cmd, { data }, data_size, delay}
        /**** CMD_Page 1 ****/
        {0xFF, (uint8_t[]){0x98, 0x81, 0x01}, 3,
         0},                             // Page Select Command - Switch to Command Page 1
        {0xB7, (uint8_t[]){0x03}, 1, 0}, // DSI Control - Set 2 lane mode

        /**** CMD_Page 3 ****/
        {0xFF, (uint8_t[]){0x98, 0x81, 0x03}, 3,
         0},                             // Page Select Command - Switch to Command Page 3
        {0x01, (uint8_t[]){0x00}, 1, 0}, // GIP_1 - Gate driver control 1
        {0x02, (uint8_t[]){0x00}, 1, 0}, // GIP_2 - Gate driver control 2
        {0x03, (uint8_t[]){0x73}, 1, 0}, // GIP_3 - Gate driver control 3
        {0x04, (uint8_t[]){0x00}, 1, 0}, // GIP_4 - Gate driver control 4
        {0x05, (uint8_t[]){0x00}, 1, 0}, // GIP_5 - Gate driver control 5
        {0x06, (uint8_t[]){0x08}, 1, 0}, // GIP_6 - Gate driver control 6
        {0x07, (uint8_t[]){0x00}, 1, 0}, // GIP_7 - Gate driver control 7
        {0x08, (uint8_t[]){0x00}, 1, 0}, // GIP_8 - Gate driver control 8
        {0x09, (uint8_t[]){0x1B}, 1, 0}, // GIP_9 - Gate driver control 9
        {0x0a, (uint8_t[]){0x01}, 1, 0}, // GIP_10 - Gate driver control 10
        {0x0b, (uint8_t[]){0x01}, 1, 0}, // GIP_11 - Gate driver control 11
        {0x0c, (uint8_t[]){0x0D}, 1, 0}, // GIP_12 - Gate driver control 12
        {0x0d, (uint8_t[]){0x01}, 1, 0}, // GIP_13 - Gate driver control 13
        {0x0e, (uint8_t[]){0x01}, 1, 0}, // GIP_14 - Gate driver control 14
        {0x0f, (uint8_t[]){0x26}, 1, 0}, // GIP_15 - Gate driver control 15
        {0x10, (uint8_t[]){0x26}, 1, 0}, // GIP_16 - Gate driver control 16
        {0x11, (uint8_t[]){0x00}, 1, 0}, // GIP_17 - Gate driver control 17
        {0x12, (uint8_t[]){0x00}, 1, 0}, // GIP_18 - Gate driver control 18
        {0x13, (uint8_t[]){0x02}, 1, 0}, // GIP_19 - Gate driver control 19
        {0x14, (uint8_t[]){0x00}, 1, 0}, // GIP_20 - Gate driver control 20
        {0x15, (uint8_t[]){0x00}, 1, 0}, // GIP_21 - Gate driver control 21
        {0x16, (uint8_t[]){0x00}, 1, 0}, // GIP_22 - Gate driver control 22
        {0x17, (uint8_t[]){0x00}, 1, 0}, // GIP_23 - Gate driver control 23
        {0x18, (uint8_t[]){0x00}, 1, 0}, // GIP_24 - Gate driver control 24
        {0x19, (uint8_t[]){0x00}, 1, 0}, // GIP_25 - Gate driver control 25
        {0x1a, (uint8_t[]){0x00}, 1, 0}, // GIP_26 - Gate driver control 26
        {0x1b, (uint8_t[]){0x00}, 1, 0}, // GIP_27 - Gate driver control 27
        {0x1c, (uint8_t[]){0x00}, 1, 0}, // GIP_28 - Gate driver control 28
        {0x1d, (uint8_t[]){0x00}, 1, 0}, // GIP_29 - Gate driver control 29
        {0x1e, (uint8_t[]){0x40}, 1, 0}, // GIP_30 - Gate driver control 30
        {0x1f, (uint8_t[]){0x00}, 1, 0}, // GIP_31 - Gate driver control 31
        {0x20, (uint8_t[]){0x06}, 1, 0}, // GIP_32 - Gate driver control 32
        {0x21, (uint8_t[]){0x01}, 1, 0}, // GIP_33 - Gate driver control 33
        {0x22, (uint8_t[]){0x00}, 1, 0}, // GIP_34 - Gate driver control 34
        {0x23, (uint8_t[]){0x00}, 1, 0}, // GIP_35 - Gate driver control 35
        {0x24, (uint8_t[]){0x00}, 1, 0}, // GIP_36 - Gate driver control 36
        {0x25, (uint8_t[]){0x00}, 1, 0}, // GIP_37 - Gate driver control 37
        {0x26, (uint8_t[]){0x00}, 1, 0}, // GIP_38 - Gate driver control 38
        {0x27, (uint8_t[]){0x00}, 1, 0}, // GIP_39 - Gate driver control 39
        {0x28, (uint8_t[]){0x33}, 1, 0}, // GIP_40 - Source timing control 1
        {0x29, (uint8_t[]){0x03}, 1, 0}, // GIP_41 - Source timing control 2
        {0x2a, (uint8_t[]){0x00}, 1, 0}, // GIP_42 - Source timing control 3
        {0x2b, (uint8_t[]){0x00}, 1, 0}, // GIP_43 - Source timing control 4
        {0x2c, (uint8_t[]){0x00}, 1, 0}, // GIP_44 - Source timing control 5
        {0x2d, (uint8_t[]){0x00}, 1, 0}, // GIP_45 - Source timing control 6
        {0x2e, (uint8_t[]){0x00}, 1, 0}, // GIP_46 - Source timing control 7
        {0x2f, (uint8_t[]){0x00}, 1, 0}, // GIP_47 - Source timing control 8
        {0x30, (uint8_t[]){0x00}, 1, 0}, // GIP_48 - Source timing control 9
        {0x31, (uint8_t[]){0x00}, 1, 0}, // GIP_49 - Source timing control 10
        {0x32, (uint8_t[]){0x00}, 1, 0}, // GIP_50 - Source timing control 11
        {0x33, (uint8_t[]){0x00}, 1, 0}, // GIP_51 - Source timing control 12
        {0x34, (uint8_t[]){0x00}, 1, 0}, // GIP_52 - Source timing control 13
        {0x35, (uint8_t[]){0x00}, 1, 0}, // GIP_53 - Source timing control 14
        {0x36, (uint8_t[]){0x00}, 1, 0}, // GIP_54 - Source timing control 15
        {0x37, (uint8_t[]){0x00}, 1, 0}, // GIP_55 - Source timing control 16
        {0x38, (uint8_t[]){0x00}, 1, 0}, // GIP_56 - Source timing control 17
        {0x39, (uint8_t[]){0x00}, 1, 0}, // GIP_57 - Source timing control 18
        {0x3a, (uint8_t[]){0x00}, 1, 0}, // GIP_58 - Source timing control 19
        {0x3b, (uint8_t[]){0x00}, 1, 0}, // GIP_59 - Source timing control 20
        {0x3c, (uint8_t[]){0x00}, 1, 0}, // GIP_60 - Source timing control 21
        {0x3d, (uint8_t[]){0x00}, 1, 0}, // GIP_61 - Source timing control 22
        {0x3e, (uint8_t[]){0x00}, 1, 0}, // GIP_62 - Source timing control 23
        {0x3f, (uint8_t[]){0x00}, 1, 0}, // GIP_63 - Source timing control 24
        {0x40, (uint8_t[]){0x00}, 1, 0}, // GIP_64 - Source timing control 25
        {0x41, (uint8_t[]){0x00}, 1, 0}, // GIP_65 - Source timing control 26
        {0x42, (uint8_t[]){0x00}, 1, 0}, // GIP_66 - Source timing control 27
        {0x43, (uint8_t[]){0x00}, 1, 0}, // GIP_67 - Source timing control 28
        {0x44, (uint8_t[]){0x00}, 1, 0}, // GIP_68 - Source timing control 29

        {0x50, (uint8_t[]){0x01}, 1, 0}, // GIP_R_L1 - Forward scan signal output 1
        {0x51, (uint8_t[]){0x23}, 1, 0}, // GIP_R_L2 - Forward scan signal output 2
        {0x52, (uint8_t[]){0x45}, 1, 0}, // GIP_R_L3 - Forward scan signal output 3
        {0x53, (uint8_t[]){0x67}, 1, 0}, // GIP_R_L4 - Forward scan signal output 4
        {0x54, (uint8_t[]){0x89}, 1, 0}, // GIP_R_L5 - Forward scan signal output 5
        {0x55, (uint8_t[]){0xab}, 1, 0}, // GIP_R_L6 - Forward scan signal output 6
        {0x56, (uint8_t[]){0x01}, 1, 0}, // GIP_R_L7 - Forward scan signal output 7
        {0x57, (uint8_t[]){0x23}, 1, 0}, // GIP_R_L8 - Forward scan signal output 8
        {0x58, (uint8_t[]){0x45}, 1, 0}, // GIP_R_L9 - Forward scan signal output 9
        {0x59, (uint8_t[]){0x67}, 1, 0}, // GIP_R_L10 - Forward scan signal output 10
        {0x5a, (uint8_t[]){0x89}, 1, 0}, // GIP_R_L11 - Forward scan signal output 11
        {0x5b, (uint8_t[]){0xab}, 1, 0}, // GIP_R_L12 - Forward scan signal output 12
        {0x5c, (uint8_t[]){0xcd}, 1, 0}, // GIP_R_L13 - Forward scan signal output 13
        {0x5d, (uint8_t[]){0xef}, 1, 0}, // GIP_R_L14 - Forward scan signal output 14

        {0x5e, (uint8_t[]){0x11}, 1, 0}, // GIP_L_L1 - Backward scan signal output 1
        {0x5f, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L2 - Backward scan signal output 2
        {0x60, (uint8_t[]){0x00}, 1, 0}, // GIP_L_L3 - Backward scan signal output 3
        {0x61, (uint8_t[]){0x07}, 1, 0}, // GIP_L_L4 - Backward scan signal output 4
        {0x62, (uint8_t[]){0x06}, 1, 0}, // GIP_L_L5 - Backward scan signal output 5
        {0x63, (uint8_t[]){0x0E}, 1, 0}, // GIP_L_L6 - Backward scan signal output 6
        {0x64, (uint8_t[]){0x0F}, 1, 0}, // GIP_L_L7 - Backward scan signal output 7
        {0x65, (uint8_t[]){0x0C}, 1, 0}, // GIP_L_L8 - Backward scan signal output 8
        {0x66, (uint8_t[]){0x0D}, 1, 0}, // GIP_L_L9 - Backward scan signal output 9
        {0x67, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L10 - Backward scan signal output 10
        {0x68, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L11 - Backward scan signal output 11
        {0x69, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L12 - Backward scan signal output 12
        {0x6a, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L13 - Backward scan signal output 13
        {0x6b, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L14 - Backward scan signal output 14
        {0x6c, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L15 - Backward scan signal output 15
        {0x6d, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L16 - Backward scan signal output 16
        {0x6e, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L17 - Backward scan signal output 17
        {0x6f, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L18 - Backward scan signal output 18
        {0x70, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L19 - Backward scan signal output 19
        {0x71, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L20 - Backward scan signal output 20
        {0x72, (uint8_t[]){0x02}, 1, 0}, // GIP_L_L21 - Backward scan signal output 21
        {0x73, (uint8_t[]){0x05}, 1, 0}, // GIP_L_L22 - Backward scan signal output 22
        {0x74, (uint8_t[]){0x01}, 1, 0}, // GIP_R_R1 - Right side signal output 1
        {0x75, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R2 - Right side signal output 2
        {0x76, (uint8_t[]){0x00}, 1, 0}, // GIP_R_R3 - Right side signal output 3
        {0x77, (uint8_t[]){0x07}, 1, 0}, // GIP_R_R4 - Right side signal output 4
        {0x78, (uint8_t[]){0x06}, 1, 0}, // GIP_R_R5 - Right side signal output 5
        {0x79, (uint8_t[]){0x0E}, 1, 0}, // GIP_R_R6 - Right side signal output 6
        {0x7a, (uint8_t[]){0x0F}, 1, 0}, // GIP_R_R7 - Right side signal output 7
        {0x7b, (uint8_t[]){0x0C}, 1, 0}, // GIP_R_R8 - Right side signal output 8
        {0x7c, (uint8_t[]){0x0D}, 1, 0}, // GIP_R_R9 - Right side signal output 9
        {0x7d, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R10 - Right side signal output 10
        {0x7e, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R11 - Right side signal output 11
        {0x7f, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R12 - Right side signal output 12
        {0x80, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R13 - Right side signal output 13
        {0x81, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R14 - Right side signal output 14
        {0x82, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R15 - Right side signal output 15
        {0x83, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R16 - Right side signal output 16
        {0x84, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R17 - Right side signal output 17
        {0x85, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R18 - Right side signal output 18
        {0x86, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R19 - Right side signal output 19
        {0x87, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R20 - Right side signal output 20
        {0x88, (uint8_t[]){0x02}, 1, 0}, // GIP_R_R21 - Right side signal output 21
        {0x89, (uint8_t[]){0x05}, 1, 0}, // GIP_R_R22 - Right side signal output 22
        {0x8A, (uint8_t[]){0x01}, 1, 0}, // GIP_EQ - Gate equalization control

        /**** CMD_Page 4 ****/
        {0xFF, (uint8_t[]){0x98, 0x81, 0x04}, 3,
         0},                             // Page Select Command - Switch to Command Page 4
        {0x38, (uint8_t[]){0x01}, 1, 0}, // VREG2OUT - VREG2 output enable
        {0x39, (uint8_t[]){0x00}, 1, 0}, // VREG1OUT - VREG1 output control
        {0x6C, (uint8_t[]){0x15}, 1, 0}, // VGH_CLAMP - VGH clamp voltage setting
        {0x6E, (uint8_t[]){0x1A}, 1, 0}, // VGL_CLAMP - VGL clamp voltage setting
        {0x6F, (uint8_t[]){0x25}, 1, 0}, // PUMP_CLAMP - Charge pump clamp setting
        {0x3A, (uint8_t[]){0xA4}, 1, 0}, // POWER_CTRL - Power control setting
        {0x8D, (uint8_t[]){0x20}, 1, 0}, // VCL - VCL voltage level setting
        {0x87, (uint8_t[]){0xBA}, 1, 0}, // VCORE_VOLT - VCORE voltage setting
        {0x3B, (uint8_t[]){0x98}, 1, 0}, // VGH_VGL_CTRL - VGH/VGL timing control

        /**** CMD_Page 1 ****/
        {0xFF, (uint8_t[]){0x98, 0x81, 0x01}, 3,
         0},                             // Page Select Command - Switch to Command Page 1
        {0x22, (uint8_t[]){0x0A}, 1, 0}, // MIPI_CTRL - MIPI interface control
        {0x31, (uint8_t[]){0x00}, 1, 0}, // INV_CTRL1 - Inversion control 1
        {0x50, (uint8_t[]){0x6B}, 1, 0}, // VREG_CTRL1 - VREG control 1
        {0x51, (uint8_t[]){0x66}, 1, 0}, // VREG_CTRL2 - VREG control 2
        {0x53, (uint8_t[]){0x73}, 1, 0}, // VREG_CTRL3 - VREG control 3
        {0x55, (uint8_t[]){0x8B}, 1, 0}, // VREG_CTRL4 - VREG control 4
        {0x60, (uint8_t[]){0x1B}, 1, 0}, // BIAS_CTRL - Bias current control
        {0x61, (uint8_t[]){0x01}, 1, 0}, // BIAS_CTRL2 - Bias control 2
        {0x62, (uint8_t[]){0x0C}, 1, 0}, // BIAS_CTRL3 - Bias control 3
        {0x63, (uint8_t[]){0x00}, 1, 0}, // BIAS_CTRL4 - Bias control 4

        // Gamma P - Positive Gamma Correction Settings
        {0xA0, (uint8_t[]){0x00}, 1, 0}, // GMCTR_P1 - Positive gamma control 1
        {0xA1, (uint8_t[]){0x15}, 1, 0}, // GMCTR_P2 - Positive gamma control 2
        {0xA2, (uint8_t[]){0x1F}, 1, 0}, // GMCTR_P3 - Positive gamma control 3
        {0xA3, (uint8_t[]){0x13}, 1, 0}, // GMCTR_P4 - Positive gamma control 4
        {0xA4, (uint8_t[]){0x11}, 1, 0}, // GMCTR_P5 - Positive gamma control 5
        {0xA5, (uint8_t[]){0x21}, 1, 0}, // GMCTR_P6 - Positive gamma control 6
        {0xA6, (uint8_t[]){0x17}, 1, 0}, // GMCTR_P7 - Positive gamma control 7
        {0xA7, (uint8_t[]){0x1B}, 1, 0}, // GMCTR_P8 - Positive gamma control 8
        {0xA8, (uint8_t[]){0x6B}, 1, 0}, // GMCTR_P9 - Positive gamma control 9
        {0xA9, (uint8_t[]){0x1E}, 1, 0}, // GMCTR_P10 - Positive gamma control 10
        {0xAA, (uint8_t[]){0x2B}, 1, 0}, // GMCTR_P11 - Positive gamma control 11
        {0xAB, (uint8_t[]){0x5D}, 1, 0}, // GMCTR_P12 - Positive gamma control 12
        {0xAC, (uint8_t[]){0x19}, 1, 0}, // GMCTR_P13 - Positive gamma control 13
        {0xAD, (uint8_t[]){0x14}, 1, 0}, // GMCTR_P14 - Positive gamma control 14
        {0xAE, (uint8_t[]){0x4B}, 1, 0}, // GMCTR_P15 - Positive gamma control 15
        {0xAF, (uint8_t[]){0x1D}, 1, 0}, // GMCTR_P16 - Positive gamma control 16
        {0xB0, (uint8_t[]){0x27}, 1, 0}, // GMCTR_P17 - Positive gamma control 17
        {0xB1, (uint8_t[]){0x49}, 1, 0}, // GMCTR_P18 - Positive gamma control 18
        {0xB2, (uint8_t[]){0x5D}, 1, 0}, // GMCTR_P19 - Positive gamma control 19
        {0xB3, (uint8_t[]){0x39}, 1, 0}, // GMCTR_P20 - Positive gamma control 20

        // Gamma N - Negative Gamma Correction Settings
        {0xC0, (uint8_t[]){0x00}, 1, 0}, // GMCTR_N1 - Negative gamma control 1
        {0xC1, (uint8_t[]){0x01}, 1, 0}, // GMCTR_N2 - Negative gamma control 2
        {0xC2, (uint8_t[]){0x0C}, 1, 0}, // GMCTR_N3 - Negative gamma control 3
        {0xC3, (uint8_t[]){0x11}, 1, 0}, // GMCTR_N4 - Negative gamma control 4
        {0xC4, (uint8_t[]){0x15}, 1, 0}, // GMCTR_N5 - Negative gamma control 5
        {0xC5, (uint8_t[]){0x28}, 1, 0}, // GMCTR_N6 - Negative gamma control 6
        {0xC6, (uint8_t[]){0x1B}, 1, 0}, // GMCTR_N7 - Negative gamma control 7
        {0xC7, (uint8_t[]){0x1C}, 1, 0}, // GMCTR_N8 - Negative gamma control 8
        {0xC8, (uint8_t[]){0x62}, 1, 0}, // GMCTR_N9 - Negative gamma control 9
        {0xC9, (uint8_t[]){0x1C}, 1, 0}, // GMCTR_N10 - Negative gamma control 10
        {0xCA, (uint8_t[]){0x29}, 1, 0}, // GMCTR_N11 - Negative gamma control 11
        {0xCB, (uint8_t[]){0x60}, 1, 0}, // GMCTR_N12 - Negative gamma control 12
        {0xCC, (uint8_t[]){0x16}, 1, 0}, // GMCTR_N13 - Negative gamma control 13
        {0xCD, (uint8_t[]){0x17}, 1, 0}, // GMCTR_N14 - Negative gamma control 14
        {0xCE, (uint8_t[]){0x4A}, 1, 0}, // GMCTR_N15 - Negative gamma control 15
        {0xCF, (uint8_t[]){0x23}, 1, 0}, // GMCTR_N16 - Negative gamma control 16
        {0xD0, (uint8_t[]){0x24}, 1, 0}, // GMCTR_N17 - Negative gamma control 17
        {0xD1, (uint8_t[]){0x4F}, 1, 0}, // GMCTR_N18 - Negative gamma control 18
        {0xD2, (uint8_t[]){0x5F}, 1, 0}, // GMCTR_N19 - Negative gamma control 19
        {0xD3, (uint8_t[]){0x39}, 1, 0}, // GMCTR_N20 - Negative gamma control 20

        /**** CMD_Page 0 ****/
        {0xFF, (uint8_t[]){0x98, 0x81, 0x00}, 3,
         0}, // Page Select Command - Switch to Command Page 0 (User Command Set)
        {0x35, (uint8_t[]){0x00}, 0,
         0}, // TE (Tearing Effect Line) ON - Enable tearing effect output signal
        // {0x11, (uint8_t []){0x00}, 0}, // SLPOUT - Sleep Out (commented out - handled by ESP-LCD
        // driver)
        {0xFE, (uint8_t[]){0x00}, 0, 0}, // NOP - No operation (custom/extended command)
        {0x29, (uint8_t[]){0x00}, 0, 0}, // DISPON - Display ON
                                         //============ Gamma END===========
};
