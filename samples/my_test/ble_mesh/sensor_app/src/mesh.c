#include <sys/printk.h>
#include <settings/settings.h>
#include <nrf.h>
#include<sys/byteorder.h>
#include <device.h>

#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/mesh.h>

#include <stdio.h>
#include <logging/log.h>

#include "pherphs.h"
#include "mesh.h"

int update_cb(struct bt_mesh_model *mod){
    struct bt_mesh_model *mod_cli;
    struct bt_mesh_model_pub *pub_cli;
    mod_cli = mod;
    pub_cli = mod_cli->pub;
    bt_mesh_model_msg_init(pub_cli->msg, BT_MESH_MODEL_OP_SENS_GET);
    net_buf_simple_add_le16(pub_cli->msg,SENS_PROP_ID_TEMP_CELSIUS);
    return 0;
}

BT_MESH_HEALTH_PUB_DEFINE(health_pub,0);
BT_MESH_MODEL_PUB_DEFINE(sensor_pub_srv,NULL,1+14);
BT_MESH_MODEL_PUB_DEFINE(sensor_pub_cli, update_cb,2+2);
BT_MESH_MODEL_PUB_DEFINE(sensor_pub_srv_s_0,NULL,1+14);
BT_MESH_MODEL_PUB_DEFINE(sensor_pub_cli_s_0,update_cb,2+2);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv, NULL, 2 + 2);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_cli, NULL, 2 + 2);

static struct bt_mesh_cfg_srv cfg_srv = {
        .relay = BT_MESH_RELAY_ENABLED,
        .beacon = BT_MESH_BEACON_ENABLED,

    #if defined(CONFIG_BT_MESH_FRIEND)
        .frnd = BT_MESH_FRIEND_ENABLED,
    #else 
        .frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
    #endif

    #if defined(CONFIG_BT_MESH_GATT_PROXY)
        .gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
    #else
        .gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
    #endif

        .default_ttl = DEFAULT_TTL,

        .net_transmit = BT_MESH_TRANSMIT(2,20),
        .relay_retransmit = BT_MESH_TRANSMIT(2,20),
};

static struct bt_mesh_cfg_cli cfg_cli = {};

static struct bt_mesh_health_srv health_srv ={};

static void sensor_desc_get( struct bt_mesh_model *model,
                                                            struct bt_mesh_msg_ctx *ctx,
                                                            struct net_buf_simple *buf)
{
    
}

static void sens_data_fill(struct net_buf_simple *msg)
{
    struct sensor_hdr_b temp, humi;
    u16_t temp_degrees[2];
    u16_t humi_percent[2];

    temp.format = SENSOR_HDR_B;
    temp.length = sizeof(temp_degrees[0])*2;
    temp.prop_id = SENS_PROP_ID_UNIT_TEMP_CELSIUS;

    humi.format = SENSOR_HDR_B;
    humi.length = sizeof(humi_percent[0])*2;
    humi.prop_id = SENS_PROP_ID_UNIT_PERCENTAGE;

    get_dht22_data(temp_degrees,humi_percent);
    
    net_buf_simple_add_mem(msg,&temp,sizeof(temp));
    net_buf_simple_add_le16(msg,temp_degrees[0]);
    net_buf_simple_add_le16(msg,temp_degrees[1]);
     net_buf_simple_add_mem(msg,&humi,sizeof(humi));
    net_buf_simple_add_le16(msg,humi_percent[0]);
    net_buf_simple_add_le16(msg,humi_percent[1]);

}

static void sens_unkown_fill(u16_t id, struct net_buf_simple *msg){

        struct sensor_hdr_a hdr;


        hdr.format = SENSOR_HDR_A;
        hdr.length = 0U;
        hdr.prop_id = id;

        net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));
}

static void sensor_create_status(u16_t id, struct net_buf_simple *msg){
    bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS);

    switch(id){
        case SENS_PROP_ID_TEMP_CELSIUS:
            sens_data_fill(msg);
            break;
        default:
            sens_unkown_fill(id,msg);
            break;
    }
}

static void sensor_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf){
    NET_BUF_SIMPLE_DEFINE(msg, 1+ MAX_SENS_STATUS_LEN+4);
    u16_t sensor_id;
    sensor_id = net_buf_simple_pull_le16(buf);
    sensor_create_status(sensor_id,&msg);
    if(bt_mesh_model_send(model, ctx, &msg, NULL, NULL)){
        printk("Unble to send Sensor get status response\n");
    }
}

static void sensor_col_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf){

}

static void sensor_series_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf){


}

static void sensor_status(struct bt_mesh_model *model, struct bt_mesh_msg_ctx  *ctx, struct net_buf_simple *buf){
    u16_t temp[2]={0}, humi[2]={0},prop_id = 0;
    u8_t length = 0;

    while(buf->len != 0){
        length = net_buf_simple_pull_u8(buf);
        if(length&128){
            length = length-128;
            prop_id = net_buf_simple_pull_le16(buf);
            switch(prop_id){
                case SENS_PROP_ID_UNIT_PERCENTAGE:
                    humi[0] = net_buf_simple_pull_le16(buf);
                    humi[1] = net_buf_simple_pull_le16(buf);
                    break;
                case SENS_PROP_ID_UNIT_TEMP_CELSIUS:
                    temp[0] = net_buf_simple_pull_le16(buf);
                    temp[1] = net_buf_simple_pull_le16(buf);  
                    break;
                default:
                    break;
            }
        }
        else{
            net_buf_simple_pull_mem(buf,buf->len);
        }
    }
    printk("Temp : %d.%d Humi : %d.%d\n",temp[0],temp[1],humi[0],humi[1]);

}

static void sensor_desc_status(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf){

}
/*
 * Generic OnOff Model Server Message Handlers
 *
 * Mesh Model Specification 3.1.1
 *
 */
static void gen_onoff_get(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	NET_BUF_SIMPLE_DEFINE(msg, 2 + 1 + 4);
	struct onoff_state *onoff_state = model->user_data;

	printk("addr 0x%04x onoff 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, onoff_state->current);
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
	net_buf_simple_add_u8(&msg, onoff_state->current);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("Unable to send On Off Status response\n");
	}
}

static void gen_onoff_set_unack(struct bt_mesh_model *model,
				struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf)
{
	struct net_buf_simple *msg = model->pub->msg;
	struct onoff_state *onoff_state = model->user_data;
	int err;

	onoff_state->current = net_buf_simple_pull_u8(buf);
	printk("addr 0x%02x state 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, onoff_state->current);

    gpio_pin_set(onoff_state->led_device, onoff_state->led_gpio_pin,
		     onoff_state->current);

	/*
	 * If a server has a publish address, it is required to
	 * publish status on a state change
	 *
	 * See Mesh Profile Specification 3.7.6.1.2
	 *
	 * Only publish if there is an assigned address
	 */

	if (onoff_state->previous != onoff_state->current &&
	    model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		printk("publish last 0x%02x cur 0x%02x\n",
		       onoff_state->previous, onoff_state->current);
		onoff_state->previous = onoff_state->current;
		bt_mesh_model_msg_init(msg,
				       BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
		net_buf_simple_add_u8(msg, onoff_state->current);
		err = bt_mesh_model_publish(model);
		if (err) {
			printk("bt_mesh_model_publish err %d\n", err);
		}
	}
}

static void gen_onoff_set(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	printk("gen_onoff_set\n");

	gen_onoff_set_unack(model, ctx, buf);
	gen_onoff_get(model, ctx, buf);
}

static void gen_onoff_status(struct bt_mesh_model *model,
			     struct bt_mesh_msg_ctx *ctx,
			     struct net_buf_simple *buf)
{
	u8_t	state;

	state = net_buf_simple_pull_u8(buf);

	printk("Node 0x%04x OnOff status from 0x%04x with state 0x%02x\n",
	       bt_mesh_model_elem(model)->addr, ctx->addr, state);
}

static const struct bt_mesh_model_op sensor_srv_op[] = {
    { BT_MESH_MODEL_OP_SENS_DESC_GET , 0, sensor_desc_get},
    { BT_MESH_MODEL_OP_SENS_GET, 0, sensor_get},
    { BT_MESH_MODEL_OP_SENS_COL_GET, 2,sensor_col_get},
    { BT_MESH_MODEL_OP_SENS_SERIES_GET, 2, sensor_series_get },
    BT_MESH_MODEL_OP_END
};

static const struct bt_mesh_model_op sensor_cli_op[] = {
    { BT_MESH_MODEL_OP_SENS_STATUS, 2, sensor_status } ,
    {BT_MESH_MODEL_OP_SENS_DESC_STATUS, 0, sensor_desc_status } ,
    BT_MESH_MODEL_OP_END
};

static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
	{ BT_MESH_MODEL_OP_GEN_ONOFF_GET, 0, gen_onoff_get },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET, 2, gen_onoff_set },
	{ BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK, 2, gen_onoff_set_unack },
	BT_MESH_MODEL_OP_END,
};

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
	{ BT_MESH_MODEL_OP_GEN_ONOFF_STATUS, 1, gen_onoff_status },
	BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV(&cfg_srv),
    BT_MESH_MODEL_CFG_CLI(&cfg_cli),
    BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV, sensor_srv_op,&sensor_pub_srv,NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_CLI,sensor_cli_op,&sensor_pub_cli,NULL)
};

static struct bt_mesh_model secondary_0_models[] = {
    BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV, sensor_srv_op, &sensor_pub_srv_s_0,NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_CLI, sensor_cli_op,&sensor_pub_cli_s_0, NULL),
    BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
		      &gen_onoff_pub_srv, &onoff_state[0]),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
		      &gen_onoff_pub_cli, &onoff_state[0])
};

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
    BT_MESH_ELEM(0, secondary_0_models, BT_MESH_MODEL_NONE)
};

static const struct bt_mesh_comp comp = {
    .cid = BT_COMP_ID_LF,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

static u16_t primary_addr;
static u16_t primary_net_idx;

static int output_number(bt_mesh_output_action_t action, u32_t number){
    printk("OOB Number %06d\n",number);
    return 0;
}

static int output_string (const char *str){
    printk("OOB String %s\n",str);
    return 0;
}

static void prov_complete(u16_t net_idx, u16_t addr){
    printk("provisioning complete for net_idx 0x%04x addr 0x%04x\n",net_idx, addr);
    primary_addr = addr;
    primary_net_idx = net_idx;
}

static void prov_reset(void){
    bt_mesh_prov_enable(BT_MESH_PROV_ADV|BT_MESH_PROV_GATT);
}

static u8_t dev_uuid[16] = { 0xdd,0xdd};

static const struct bt_mesh_prov prov = {
    .uuid = dev_uuid,

    .output_size = 6,
    .output_actions = (BT_MESH_DISPLAY_NUMBER | BT_MESH_DISPLAY_STRING),
    .output_number = output_number,
    .output_string = output_string,
    .complete = prov_complete,
    .reset = prov_reset,
};

static u8_t trans_id;
void send_unack(u8_t onoff){
    struct bt_mesh_model *mod_cli, *mod_srv;
	struct bt_mesh_model_pub *pub_cli, *pub_srv;
	int err;

	mod_cli = &secondary_0_models[3];
	pub_cli = mod_cli->pub;

	mod_srv =&secondary_0_models[2];
	pub_srv = mod_srv->pub;

	/* If unprovisioned, just call the set function.
	 * The intent is to have switch-like behavior
	 * prior to provisioning. Once provisioned,
	 * the button and its corresponding led are no longer
	 * associated and act independently. So, if a button is to
	 * control its associated led after provisioning, the button
	 * must be configured to either publish to the led's unicast
	 * address or a group to which the led is subscribed.
	 */

	if (primary_addr == BT_MESH_ADDR_UNASSIGNED) {
		NET_BUF_SIMPLE_DEFINE(msg, 1);
		struct bt_mesh_msg_ctx ctx = {
			.addr = 0 + primary_addr,
		};

		/* This is a dummy message sufficient
		 * for the led server
		 */

		net_buf_simple_add_u8(&msg, onoff);
		gen_onoff_set_unack(mod_srv, &ctx, &msg);
		return;
	}

	if (pub_cli->addr == BT_MESH_ADDR_UNASSIGNED) {
		return;
	}
	bt_mesh_model_msg_init(pub_cli->msg,
			       BT_MESH_MODEL_OP_GEN_ONOFF_SET);
	net_buf_simple_add_u8(pub_cli->msg, onoff);
	net_buf_simple_add_u8(pub_cli->msg, trans_id++);
	err = bt_mesh_model_publish(mod_cli);
	if (err) {
		printk("bt_mesh_model_publish err %d\n", err);
	}
}


static void bt_ready(int err){
    struct bt_le_oob oob;
    if(err){
        printk("Bluetooth init failed (err %d)\n",err);
        return;
    }

    printk("Bluetooth initialized\n");

    err = bt_mesh_init(&prov, &comp);
    if(err){
        printk("Initializing mesh failed (err %d) \n", err);
        return;
    }
    if(IS_ENABLED(CONFIG_SETTINGS)){
        settings_load();
    }

    if(bt_le_oob_get_local(BT_ID_DEFAULT, &oob)){
        printk("Identity Address unavailable\n");
    }else{
        memcpy(dev_uuid,oob.addr.a.val, 6);
    }

    bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
    printk("Mesh initialized \n");
}

int mesh_init(){
    return bt_enable(bt_ready);
}