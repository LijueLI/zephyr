#define  DEFAULT_TTL 31
#define MAX_SENS_STATUS_LEN 14


#define SENS_PROP_ID_TEMP_CELSIUS 0x2A1F
#define SENS_PROP_ID_HUMI 0x2A6F



#define SENS_PROP_ID_UNIT_PERCENTAGE 0x27AD
#define SENS_PROP_ID_UNIT_TEMP_CELSIUS 0x272F


#define BT_MESH_MODEL_OP_SENS_DESC_GET BT_MESH_MODEL_OP_2(0x82,0x30)
#define BT_MESH_MODEL_OP_SENS_GET BT_MESH_MODEL_OP_2(0x82,0x31)
#define BT_MESH_MODEL_OP_SENS_COL_GET BT_MESH_MODEL_OP_2(0x82,0x32)
#define BT_MESH_MODEL_OP_SENS_SERIES_GET BT_MESH_MODEL_OP_2(0x82,0x33)
#define BT_MESH_MODEL_OP_GEN_ONOFF_GET		BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET		BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS	BT_MESH_MODEL_OP_2(0x82, 0x04)

#define BT_MESH_MODEL_OP_SENS_DESC_STATUS BT_MESH_MODEL_OP_1(0x51)
#define BT_MESH_MODEL_OP_SENS_STATUS BT_MESH_MODEL_OP_1(0x52)


enum{
    SENSOR_HDR_A = 0,
    SENSOR_HDR_B = 1,
};

struct sensor_hdr_a{
    u16_t prop_id : 11;
    u16_t length : 4;
    u16_t format : 1; 
}__packed;

struct sensor_hdr_b {
    u8_t length : 7;
    u8_t format : 1;
    u16_t prop_id;
}__packed;

int mesh_init(void);

void send_unack(u8_t onoff);