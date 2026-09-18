#ifndef __ARM_GRAVITY_H
#define __ARM_GRAVITY_H

/*
 * ¼ç¹Ø½Ú/Öâ¹Ø½Ú¾²Ì¬ÖØÁ¦Ç°À¡¡£
 * ÕâÐ© volatile ±äÁ¿¶¼¿ÉÔÚ Keil Watch ´°¿ÚÖ±½ÓÐÞ¸Ä¡£
 */
extern volatile float arm_gravity_scale;          /* ×Ü²¹³¥±ÈÀý£¬1.0=°´Ä£ÐÍÂú²¹³¥ */
extern volatile float arm_gravity_shoulder_dir;   /* ¼ç¹Ø½Ú·½Ïò£¬Ö»ÔÊÐí +1/-1 */
extern volatile float arm_gravity_elbow_dir;      /* Öâ¹Ø½Ú·½Ïò£¬Ö»ÔÊÐí +1/-1 */

/* ²Î¿¼¹¤³ÌÍâ²¿¼õËÙ±È¡£ÏÈ°´ 7.5 ²âÊÔ£¬¿ÉÔÚ Watch ÖÐÐÞ¸Ä¡£ */
extern volatile float arm_gravity_gear_sh;
extern volatile float arm_gravity_gear_el;

/* µ÷ÊÔ±äÁ¿£ºjoint Îª¹Ø½Ú²à¼ÆËãÁ¦¾Ø£¬motor Îª×îÖÕ·¢ËÍ¸ø MIT ½Ó¿ÚµÄÁ¦¾Ø¡£ */

extern volatile float arm_gravity_tau_shoulder_motor;
extern volatile float arm_gravity_tau_elbow_motor;

/*
 * ÊäÈëµ±Ç°¼ç¹Ø½Ú q1¡¢Öâ¹Ø½Ú q2£¨rad£©¡£
 * Êä³ö×îÖÕ·¢ËÍ¸ø AK80-9¡¢AK45-10 µÄµç»ú²àÇ°À¡Á¦¾Ø£¨Nm£©¡£
 */
void arm_gravity_get(float q1, float q2, float dt,
                     float *tau_shoulder_motor,
                     float *tau_elbow_motor);

/* Us‚YÍ›MˆÕ)>ûp */
extern volatile float arm_wrist_gravity_scale;

/* ¡—Us‚YÍ›Mˆ›é */
float arm_wrist_gravity_get(float q0, float q1, float q2,
                            float l3_level_c, float dt);

#endif /* __ARM_GRAVITY_H */