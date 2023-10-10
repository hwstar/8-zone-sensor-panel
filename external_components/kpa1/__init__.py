import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.components import alarm_control_panel
from esphome.const import CONF_ID

CONF_ID_ACP = "acp_id"
CONF_ENTRY_SILENT = "entry_silent"
CONF_EXIT_SILENT = "exit_silent"
CONF_ALARM_SILENT = "alarm_silent"
DEPENDENCIES = ['uart','alarm_control_panel']


kpa1_ns = cg.esphome_ns.namespace('kpa1')
Kpa1 = kpa1_ns.class_('Kpa1', cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Kpa1),
    cv.Required(CONF_ID_ACP): cv.use_id(alarm_control_panel.AlarmControlPanel),
    cv.Required(CONF_ENTRY_SILENT): cv.boolean,
    cv.Required(CONF_EXIT_SILENT): cv.boolean,
    cv.Required(CONF_ALARM_SILENT): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    
 
    acp = await cg.get_variable(config[CONF_ID_ACP])
    cg.add(var.set_acp(acp))
    
   
    cg.add(var.set_keypad_entry_silent(config[CONF_ENTRY_SILENT]))
    cg.add(var.set_keypad_exit_silent(config[CONF_EXIT_SILENT]))
    cg.add(var.set_keypad_alarm_silent(config[CONF_ALARM_SILENT]))
      
    # Initialization
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
