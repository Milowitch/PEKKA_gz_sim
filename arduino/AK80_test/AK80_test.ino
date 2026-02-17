#include <mcp_can.h>
#include <mcp_can_dfs.h>

MCP_CAN CAN1(9);

#define CAN1_INT 2

void setup() {
  Serial.begin(115200);
  if (CAN1.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.print("CAN1: Init OK!\r\n");
    CAN1.setMode(MCP_NORMAL);
  } else Serial.print("CAN1: Init Fail!!!\r\n");
}

void loop() {
}