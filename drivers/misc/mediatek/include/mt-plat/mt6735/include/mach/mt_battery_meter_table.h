/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#define BAT_NTC_10 1
#define BAT_NTC_47 0

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             16900
#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900
#endif

#define RBAT_PULL_UP_VOLT          1800

typedef struct _BATTERY_PROFILE_STRUCT {
	signed int percentage;
	signed int voltage;
} BATTERY_PROFILE_STRUCT, *BATTERY_PROFILE_STRUCT_P;

typedef struct _R_PROFILE_STRUCT {
	signed int resistance;
	signed int voltage;
} R_PROFILE_STRUCT, *R_PROFILE_STRUCT_P;

typedef enum {
	T1_0C,
	T2_25C,
	T3_50C
} PROFILE_TEMPERATURE;

#if (BAT_NTC_10 == 1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
	{-20, 68237},
	{-15, 53650},
	{-10, 42506},
	{ -5, 33892},
	{  0, 27219},
	{  5, 22021},
	{ 10, 17926},
	{ 15, 14674},
	{ 20, 12081},
	{ 25, 10000},
	{ 30, 8315},
	{ 35, 6948},
	{ 40, 5834},
	{ 45, 4917},
	{ 50, 4161},
	{ 55, 3535},
	{ 60, 3014}
};
#endif

#if (BAT_NTC_47 == 1)
	BATT_TEMPERATURE Batt_Temperature_Table[] = {
	{-20, 483954},
	{-15, 360850},
	{-10, 271697},
	{ -5, 206463},
	{  0, 158214},
	{  5, 122259},
	{ 10, 95227},
	{ 15, 74730},
	{ 20, 59065},
	{ 25, 47000},
	{ 30, 37643},
	{ 35, 30334},
	{ 40, 24591},
	{ 45, 20048},
	{ 50, 16433},
	{ 55, 13539},
	{ 60, 11210}
	};
#endif

/* T0 -10C */
// jinfeng add different macros to distinguish between S318 and S327 20170411
#if defined(CONFIG_BATEERY_PARAMETER_FOR_S327)
BATTERY_PROFILE_STRUCT battery_profile_t0[] = {
{0,4350},
{1,4326},
{2,4308},
{3,4290},
{4,4275},
{5,4261},
{6,4247},
{7,4234},
{8,4221},
{9,4209},
{10,4198},
{11,4186},
{12,4174},
{13,4163},
{14,4151},
{15,4140},
{16,4129},
{17,4119},
{18,4109},
{19,4099},
{20,4089},
{21,4079},
{22,4067},
{23,4052},
{24,4036},
{25,4020},
{26,4005},
{27,3991},
{28,3979},
{29,3968},
{30,3958},
{31,3949},
{32,3942},
{33,3934},
{34,3927},
{35,3920},
{36,3913},
{37,3907},
{38,3900},
{39,3893},
{40,3887},
{41,3880},
{42,3874},
{43,3868},
{44,3862},
{45,3857},
{46,3851},
{47,3846},
{48,3841},
{49,3836},
{50,3831},
{51,3827},
{52,3823},
{53,3819},
{54,3815},
{55,3812},
{56,3808},
{57,3805},
{58,3803},
{59,3800},
{60,3798},
{61,3795},
{62,3793},
{63,3791},
{64,3789},
{65,3786},
{66,3784},
{67,3782},
{68,3780},
{69,3778},
{70,3776},
{71,3773},
{72,3771},
{73,3768},
{74,3765},
{75,3763},
{76,3759},
{77,3756},
{78,3753},
{79,3749},
{80,3745},
{81,3741},
{82,3736},
{83,3731},
{84,3727},
{85,3722},
{86,3717},
{87,3713},
{88,3709},
{89,3704},
{90,3700},
{91,3695},
{92,3689},
{93,3681},
{94,3669},
{95,3651},
{97,3623},
{98,3583},
{98,3535},
{99,3500},
{99,3468},
{100,3439},
{100,3400},
{100,3400},
{100,3400},

};

/* T1 0C */
BATTERY_PROFILE_STRUCT battery_profile_t1[] = {
{0,4346},
{1,4327},
{2,4311},
{3,4296},
{4,4282},
{5,4270},
{6,4258},
{7,4247},
{8,4235},
{9,4224},
{10,4213},
{11,4202},
{12,4192},
{13,4181},
{14,4171},
{15,4160},
{16,4149},
{17,4139},
{18,4128},
{19,4118},
{20,4107},
{21,4098},
{22,4089},
{23,4082},
{24,4074},
{25,4065},
{26,4053},
{27,4038},
{28,4020},
{29,4002},
{30,3987},
{31,3975},
{32,3964},
{33,3955},
{34,3947},
{35,3940},
{36,3933},
{37,3926},
{38,3920},
{39,3913},
{40,3906},
{41,3899},
{42,3891},
{43,3884},
{44,3878},
{45,3871},
{46,3865},
{47,3859},
{48,3853},
{49,3848},
{50,3843},
{51,3838},
{52,3833},
{53,3828},
{54,3824},
{55,3820},
{56,3816},
{57,3812},
{58,3808},
{59,3804},
{60,3801},
{61,3798},
{62,3795},
{63,3792},
{64,3790},
{65,3787},
{66,3785},
{67,3783},
{68,3781},
{69,3779},
{70,3778},
{71,3776},
{72,3774},
{73,3772},
{74,3770},
{75,3767},
{76,3765},
{77,3762},
{78,3759},
{79,3756},
{80,3752},
{81,3748},
{82,3744},
{83,3739},
{84,3733},
{85,3728},
{86,3721},
{87,3714},
{88,3707},
{89,3702},
{90,3698},
{91,3695},
{92,3693},
{93,3689},
{94,3684},
{95,3676},
{96,3656},
{97,3619},
{98,3565},
{99,3491},
{100,3400},
{100,3400},
{100,3400},
{100,3400},
{100,3400},

};

/* T2 25C */
BATTERY_PROFILE_STRUCT battery_profile_t2[] = {
{0,4346},
{1,4329},
{2,4315},
{3,4303},
{4,4291},
{5,4279},
{6,4268},
{7,4257},
{8,4246},
{9,4235},
{10,4224},
{11,4213},
{12,4202},
{13,4191},
{14,4181},
{15,4171},
{16,4160},
{17,4150},
{18,4139},
{19,4129},
{20,4118},
{21,4108},
{22,4098},
{23,4088},
{24,4078},
{25,4071},
{26,4065},
{27,4058},
{28,4046},
{29,4031},
{30,4014},
{31,4000},
{32,3989},
{33,3980},
{34,3974},
{35,3968},
{36,3963},
{37,3956},
{38,3948},
{39,3940},
{40,3930},
{41,3920},
{42,3910},
{43,3900},
{44,3891},
{45,3883},
{46,3876},
{47,3869},
{48,3863},
{49,3857},
{50,3851},
{51,3846},
{52,3841},
{53,3836},
{54,3832},
{55,3827},
{56,3823},
{57,3819},
{58,3815},
{59,3811},
{60,3808},
{61,3804},
{62,3800},
{63,3797},
{64,3794},
{65,3791},
{66,3788},
{67,3785},
{68,3782},
{69,3779},
{70,3776},
{71,3773},
{72,3770},
{73,3767},
{74,3764},
{75,3762},
{76,3759},
{77,3755},
{78,3752},
{79,3748},
{80,3745},
{81,3742},
{82,3738},
{83,3734},
{83,3728},
{84,3722},
{85,3717},
{86,3710},
{87,3703},
{88,3695},
{89,3691},
{90,3689},
{91,3688},
{92,3686},
{93,3684},
{94,3680},
{95,3672},
{96,3648},
{97,3605},
{98,3548},
{99,3471},
{100,3400},
{100,3400},
{100,3400},
{100,3400},



};

/* T3 50C */
BATTERY_PROFILE_STRUCT battery_profile_t3[] = {
{0,4337},
{1,4323},
{2,4312},
{3,4300},
{4,4289},
{5,4277},
{6,4266},
{7,4255},
{8,4244},
{9,4233},
{10,4222},
{11,4212},
{12,4201},
{13,4190},
{14,4179},
{15,4169},
{16,4158},
{17,4148},
{18,4137},
{19,4127},
{20,4117},
{21,4107},
{22,4097},
{23,4087},
{24,4077},
{25,4068},
{26,4058},
{27,4049},
{28,4040},
{29,4031},
{30,4022},
{31,4013},
{32,4004},
{33,3996},
{34,3988},
{34,3980},
{35,3972},
{36,3964},
{37,3956},
{38,3948},
{39,3940},
{40,3932},
{41,3923},
{42,3912},
{43,3901},
{44,3890},
{45,3880},
{46,3872},
{47,3865},
{48,3859},
{49,3853},
{50,3848},
{51,3843},
{52,3838},
{53,3833},
{54,3828},
{55,3824},
{56,3820},
{57,3816},
{58,3812},
{59,3808},
{60,3805},
{61,3801},
{62,3798},
{63,3795},
{64,3792},
{65,3789},
{66,3785},
{67,3783},
{68,3780},
{69,3777},
{70,3773},
{71,3769},
{72,3764},
{73,3758},
{74,3753},
{75,3748},
{76,3744},
{77,3740},
{78,3736},
{79,3732},
{80,3728},
{81,3725},
{82,3722},
{83,3718},
{84,3713},
{85,3707},
{86,3701},
{87,3695},
{88,3688},
{89,3680},
{90,3677},
{91,3675},
{92,3675},
{93,3673},
{94,3671},
{95,3668},
{96,3660},
{97,3634},
{98,3591},
{99,3534},
{100,3457},
{100,3400},
{100,3400},
{100,3400},


};

/* battery profile for actual temperature. The size should be the same as T1, T2 and T3 */
BATTERY_PROFILE_STRUCT battery_profile_temperature[] = {
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},

};

/* T0 -10C */
R_PROFILE_STRUCT r_profile_t0[] = {
{1474,4350},
{1474,4326},
{1487,4308},
{1483,4290},
{1473,4275},
{1460,4261},
{1448,4247},
{1438,4234},
{1424,4221},
{1412,4209},
{1405,4198},
{1395,4186},
{1384,4174},
{1371,4163},
{1360,4151},
{1347,4140},
{1338,4129},
{1333,4119},
{1326,4109},
{1318,4099},
{1311,4089},
{1302,4079},
{1291,4067},
{1275,4052},
{1259,4036},
{1240,4020},
{1222,4005},
{1211,3991},
{1202,3979},
{1196,3968},
{1190,3958},
{1187,3949},
{1186,3942},
{1184,3934},
{1176,3927},
{1178,3920},
{1175,3913},
{1171,3907},
{1168,3900},
{1164,3893},
{1159,3887},
{1156,3880},
{1154,3874},
{1152,3868},
{1148,3862},
{1147,3857},
{1146,3851},
{1145,3846},
{1146,3841},
{1144,3836},
{1145,3831},
{1141,3827},
{1142,3823},
{1146,3819},
{1147,3815},
{1148,3812},
{1150,3808},
{1151,3805},
{1156,3803},
{1160,3800},
{1166,3798},
{1174,3795},
{1175,3793},
{1184,3791},
{1190,3789},
{1192,3786},
{1203,3784},
{1211,3782},
{1218,3780},
{1226,3778},
{1239,3776},
{1247,3773},
{1259,3771},
{1269,3768},
{1282,3765},
{1295,3763},
{1310,3759},
{1326,3756},
{1343,3753},
{1359,3749},
{1378,3745},
{1395,3741},
{1420,3736},
{1438,3731},
{1465,3727},
{1490,3722},
{1519,3717},
{1549,3713},
{1581,3709},
{1616,3704},
{1654,3700},
{1692,3695},
{1737,3689},
{1778,3681},
{1821,3669},
{1858,3651},
{1895,3623},
{1939,3583},
{1963,3535},
{1875,3500},
{1800,3468},
{1726,3439},
{1630,3400},
{1631,3400},
{1629,3400},

};

/* T1 0C */
R_PROFILE_STRUCT r_profile_t1[] = {
{757,4346},
{757,4327},
{761,4311},
{760,4296},
{757,4282},
{755,4270},
{751,4258},
{746,4247},
{740,4235},
{735,4224},
{731,4213},
{726,4202},
{721,4192},
{717,4181},
{713,4171},
{709,4160},
{705,4149},
{702,4139},
{696,4128},
{692,4118},
{688,4107},
{686,4098},
{684,4089},
{685,4082},
{686,4074},
{684,4065},
{678,4053},
{666,4038},
{650,4020},
{637,4002},
{627,3987},
{620,3975},
{615,3964},
{610,3955},
{608,3947},
{606,3940},
{604,3933},
{599,3926},
{597,3920},
{594,3913},
{588,3906},
{584,3899},
{581,3891},
{577,3884},
{575,3878},
{572,3871},
{571,3865},
{570,3859},
{569,3853},
{569,3848},
{567,3843},
{567,3838},
{567,3833},
{568,3828},
{568,3824},
{569,3820},
{571,3816},
{573,3812},
{573,3808},
{574,3804},
{574,3801},
{576,3798},
{580,3795},
{581,3792},
{583,3790},
{585,3787},
{588,3785},
{593,3783},
{597,3781},
{600,3779},
{604,3778},
{608,3776},
{613,3774},
{620,3772},
{626,3770},
{631,3767},
{637,3765},
{643,3762},
{650,3759},
{659,3756},
{668,3752},
{678,3748},
{688,3744},
{699,3739},
{709,3733},
{721,3728},
{734,3721},
{749,3714},
{766,3707},
{785,3702},
{808,3698},
{834,3695},
{865,3693},
{901,3689},
{944,3684},
{994,3676},
{1046,3656},
{1100,3619},
{1176,3565},
{1281,3491},
{1489,3400},
{940,3400},
{940,3400},
{940,3400},
{940,3400},

};

/* T2 25C */
R_PROFILE_STRUCT r_profile_t2[] = {
{178,4346},
{178,4329},
{180,4315},
{181,4303},
{182,4291},
{182,4279},
{183,4268},
{183,4257},
{183,4246},
{183,4235},
{184,4224},
{184,4213},
{183,4202},
{184,4191},
{184,4181},
{185,4171},
{185,4160},
{186,4150},
{187,4139},
{188,4129},
{188,4118},
{189,4108},
{190,4098},
{191,4088},
{193,4078},
{195,4071},
{198,4065},
{200,4058},
{199,4046},
{197,4031},
{194,4014},
{194,4000},
{194,3989},
{195,3980},
{198,3974},
{200,3968},
{201,3963},
{200,3956},
{198,3948},
{195,3940},
{191,3930},
{187,3920},
{182,3910},
{179,3900},
{176,3891},
{173,3883},
{171,3876},
{170,3869},
{170,3863},
{170,3857},
{169,3851},
{169,3846},
{169,3841},
{169,3836},
{169,3832},
{170,3827},
{171,3823},
{171,3819},
{171,3815},
{171,3811},
{172,3808},
{172,3804},
{172,3800},
{172,3797},
{172,3794},
{172,3791},
{172,3788},
{172,3785},
{172,3782},
{173,3779},
{173,3776},
{172,3773},
{172,3770},
{171,3767},
{171,3764},
{171,3762},
{170,3759},
{170,3755},
{170,3752},
{169,3748},
{170,3745},
{171,3742},
{171,3738},
{172,3734},
{173,3728},
{173,3722},
{175,3717},
{176,3710},
{176,3703},
{176,3695},
{176,3691},
{178,3689},
{181,3688},
{185,3686},
{191,3684},
{197,3680},
{204,3672},
{201,3648},
{203,3605},
{211,3548},
{223,3471},
{210,3400},
{210,3400},
{210,3400},
{210,3400},


};

/* T3 50C */
R_PROFILE_STRUCT r_profile_t3[] = {
{115,4340},
{170,4211},
{170,4189},
{170,4168},
{168,4147},
{170,4126},
{168,4106},
{170,4087},
{168,4067},
{165,4048},
{167,4031},
{168,4013},
{168,3995},
{170,3979},
{173,3964},
{180,3948},
{180,3929},
{168,3909},
{155,3888},
{148,3873},
{140,3860},
{140,3849},
{140,3839},
{135,3829},
{138,3821},
{138,3813},
{133,3805},
{135,3799},
{138,3793},
{143,3788},
{140,3780},
{145,3771},
{143,3760},
{138,3750},
{115,3741},
{115,3733},
{115,3725},
{118,3718},
{118,3707},
{118,3695},
{118,3681},
{118,3676},
{123,3675},
{128,3667},
{125,3619},
{128,3507},
{128,3400},
{128,3400},
{128,3400},
{128,3400},
{128,3400},
{128,3400},

};

/* r-table profile for actual temperature. The size should be the same as T1, T2 and T3 */
R_PROFILE_STRUCT r_profile_temperature[] = {
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},

};
#else
// for s318

/* T0 -10C */
BATTERY_PROFILE_STRUCT battery_profile_t0[] = {
{0,4344},
{2,4303},
{4,4270},
{6,4240},
{8,4213},
{10,4188},
{12,4163},
{14,4140},
{16,4119},
{18,4100},
{20,4076},
{23,4043},
{25,4013},
{27,3987},
{29,3966},
{31,3949},
{33,3935},
{35,3921},
{37,3908},
{39,3894},
{41,3881},
{43,3868},
{45,3857},
{47,3847},
{49,3837},
{51,3828},
{53,3821},
{55,3814},
{57,3808},
{59,3802},
{61,3796},
{63,3793},
{66,3787},
{68,3782},
{70,3778},
{72,3771},
{74,3765},
{76,3759},
{78,3752},
{80,3746},
{82,3737},
{84,3731},
{86,3722},
{88,3713},
{90,3701},
{92,3685},
{94,3662},
{96,3619},
{98,3564},
{99,3527},
{100,3496},
{100,3400},

};

/* T1 0C */
BATTERY_PROFILE_STRUCT battery_profile_t1[] = {
{0,4344},
{2,4310},
{4,4281},
{6,4255},
{8,4230},
{10,4208},
{12,4184},
{14,4162},
{16,4141},
{18,4120},
{20,4103},
{22,4085},
{24,4059},
{26,4027},
{28,3996},
{30,3974},
{32,3956},
{34,3942},
{36,3927},
{38,3912},
{40,3898},
{42,3884},
{44,3871},
{46,3860},
{48,3848},
{50,3838},
{52,3827},
{55,3820},
{57,3812},
{59,3807},
{61,3800},
{63,3795},
{65,3792},
{67,3788},
{69,3784},
{71,3779},
{73,3775},
{75,3769},
{77,3764},
{79,3757},
{81,3749},
{83,3739},
{85,3728},
{87,3714},
{89,3704},
{91,3696},
{93,3689},
{95,3672},
{97,3614},
{99,3488},
{100,3400},
{100,3400},

};

/* T2 25C */
BATTERY_PROFILE_STRUCT battery_profile_t2[] = {
{0,4346},
{2,4314},
{4,4288},
{6,4265},
{8,4242},
{10,4220},
{12,4199},
{14,4178},
{16,4157},
{18,4137},
{20,4116},
{22,4096},
{24,4078},
{26,4066},
{28,4043},
{30,4011},
{32,3988},
{34,3974},
{36,3962},
{38,3945},
{40,3926},
{42,3907},
{44,3890},
{46,3875},
{48,3863},
{50,3851},
{52,3841},
{54,3832},
{56,3823},
{58,3815},
{60,3808},
{62,3801},
{64,3794},
{66,3788},
{68,3781},
{70,3776},
{72,3770},
{74,3764},
{76,3758},
{78,3750},
{80,3743},
{82,3735},
{84,3724},
{86,3712},
{88,3697},
{90,3690},
{92,3686},
{94,3680},
{96,3640},
{98,3534},
{100,3400},
{100,3400},

};

/* T3 50C */
BATTERY_PROFILE_STRUCT battery_profile_t3[] = {
{0,4340},
{2,4211},
{4,4189},
{7,4168},
{9,4147},
{11,4126},
{13,4106},
{15,4087},
{18,4067},
{20,4048},
{22,4031},
{24,4013},
{26,3995},
{29,3979},
{31,3964},
{33,3948},
{35,3929},
{37,3909},
{40,3888},
{42,3873},
{44,3860},
{46,3849},
{48,3839},
{51,3829},
{53,3821},
{55,3813},
{57,3805},
{59,3799},
{62,3793},
{64,3788},
{66,3780},
{68,3771},
{70,3760},
{73,3750},
{75,3741},
{77,3733},
{79,3725},
{81,3718},
{84,3707},
{86,3695},
{88,3681},
{90,3676},
{92,3675},
{95,3667},
{97,3619},
{99,3507},
{100,3400},
{100,3400},
{100,3400},
{100,3400},
{100,3400},
{100,3400},

};

/* battery profile for actual temperature. The size should be the same as T1, T2 and T3 */
BATTERY_PROFILE_STRUCT battery_profile_temperature[] = {
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},

};

/* T0 -10C */
R_PROFILE_STRUCT r_profile_t0[] = {
{1077,4344},
{1735,4303},
{1718,4270},
{1673,4240},
{1633,4213},
{1600,4188},
{1565,4163},
{1530,4140},
{1508,4119},
{1475,4100},
{1455,4076},
{1400,4043},
{1368,4013},
{1340,3987},
{1335,3966},
{1323,3949},
{1328,3935},
{1328,3921},
{1320,3908},
{1308,3894},
{1308,3881},
{1308,3868},
{1300,3857},
{1298,3847},
{1300,3837},
{1303,3828},
{1305,3821},
{1308,3814},
{1323,3808},
{1330,3802},
{1333,3796},
{1350,3793},
{1373,3787},
{1383,3782},
{1400,3778},
{1428,3771},
{1448,3765},
{1475,3759},
{1498,3752},
{1538,3746},
{1573,3737},
{1618,3731},
{1660,3722},
{1718,3713},
{1778,3701},
{1838,3685},
{1908,3662},
{1995,3619},
{2038,3564},
{1948,3527},
{1868,3496},
{1635,3400},

};

/* T1 0C */
R_PROFILE_STRUCT r_profile_t1[] = {
{490,4344},
{990,4310},
{993,4281},
{983,4255},
{970,4230},
{965,4208},
{950,4184},
{935,4162},
{933,4141},
{918,4120},
{918,4103},
{910,4085},
{893,4059},
{863,4027},
{828,3996},
{815,3974},
{813,3956},
{808,3942},
{805,3927},
{793,3912},
{793,3898},
{780,3884},
{780,3871},
{770,3860},
{773,3848},
{770,3838},
{768,3827},
{775,3820},
{775,3812},
{785,3807},
{780,3800},
{793,3795},
{808,3792},
{820,3788},
{830,3784},
{843,3779},
{868,3775},
{883,3769},
{905,3764},
{930,3757},
{955,3749},
{990,3739},
{1020,3728},
{1060,3714},
{1110,3704},
{1175,3696},
{1253,3689},
{1350,3672},
{1438,3614},
{1548,3488},
{1630,3400},
{1635,3400},

};

/* T2 25C */
R_PROFILE_STRUCT r_profile_t2[] = {
{139,4346},
{195,4314},
{197,4288},
{198,4265},
{199,4242},
{199,4220},
{200,4199},
{201,4178},
{202,4157},
{204,4137},
{205,4116},
{206,4096},
{209,4078},
{216,4066},
{219,4043},
{212,4011},
{211,3988},
{215,3974},
{219,3962},
{213,3945},
{204,3926},
{195,3907},
{189,3890},
{185,3875},
{183,3863},
{183,3851},
{184,3841},
{184,3832},
{184,3823},
{186,3815},
{186,3808},
{188,3801},
{189,3794},
{189,3788},
{190,3781},
{189,3776},
{189,3770},
{191,3764},
{191,3758},
{191,3750},
{193,3743},
{196,3735},
{199,3724},
{204,3712},
{207,3697},
{213,3690},
{224,3686},
{243,3680},
{255,3640},
{279,3534},
{190,3400},
{190,3400},

};

/* T3 50C */
R_PROFILE_STRUCT r_profile_t3[] = {
{115,4340},
{170,4211},
{170,4189},
{170,4168},
{168,4147},
{170,4126},
{168,4106},
{170,4087},
{168,4067},
{165,4048},
{167,4031},
{168,4013},
{168,3995},
{170,3979},
{173,3964},
{180,3948},
{180,3929},
{168,3909},
{155,3888},
{148,3873},
{140,3860},
{140,3849},
{140,3839},
{135,3829},
{138,3821},
{138,3813},
{133,3805},
{135,3799},
{138,3793},
{143,3788},
{140,3780},
{145,3771},
{143,3760},
{138,3750},
{115,3741},
{115,3733},
{115,3725},
{118,3718},
{118,3707},
{118,3695},
{118,3681},
{118,3676},
{123,3675},
{128,3667},
{125,3619},
{128,3507},
{128,3400},
{128,3400},
{128,3400},
{128,3400},
{128,3400},
{128,3400},

};

/* r-table profile for actual temperature. The size should be the same as T1, T2 and T3 */
R_PROFILE_STRUCT r_profile_temperature[] = {
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},
{0,0},

};
#endif

/* ============================================================
// function prototype
// ============================================================*/
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUCT_P fgauge_get_profile(unsigned int temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUCT_P fgauge_get_profile_r_table(unsigned int temperature);

#endif	/*#ifndef _CUST_BATTERY_METER_TABLE_H*/

