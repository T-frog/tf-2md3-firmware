#define EXPORT_FIELDS(...)
#define EXPORT_FIELDS1(cont, type, name, ...) \
  type name;                                  \
  EXPORT_FIELDS_REPEAT(__VA_ARGS__);

#define EXPORT_FIELDS_REPEAT(cont, ...) EXPORT_FIELDS##cont(__VA_ARGS__)

#define EXPORT_STRUCT(name, ...) \
  typedef struct                 \
  {                              \
    EXPORT_FIELDS(__VA_ARGS__)   \
  } name;

EXPORT_STRUCT(
    MotorState2,
    1, int32_t, vel,
    1, int32_t, vel1,
    1, int32_t, pos,        // count
    1, uint32_t, posc,      // count
    1, uint32_t, enc_buf,   // count
    1, uint32_t, enc_buf2,  // count
    1, int32_t, spd,
    1, uint32_t, spd_cnt,
    1, uint16_t, enc,
    1, int16_t, dir)

