/*#ifndef MODULES_CANBUS_CAN_CLIENT_CLIENT_PCIE_CAN_CLIENT_H_
#define MODULES_CANBUS_CAN_CLIENT_CLIENT_PCIE_CAN_CLIENT_H_*/

#include <string>
#include <vector>
//#include "pcie_can/include/ntcan.h"
#include "gflags/gflags.h"
#include "modules/canbus/can_client/can_client.h"
#include "modules/canbus/common/canbus_consts.h"
#include "modules/canbus/proto/can_card_parameter.pb.h"
#include "modules/common/proto/error_code.pb.h"
#include "linux/can.h"
#include "linux/can/raw.h"
#include "sys/socket.h"



/**
 * @namespace apollo::canbus::can
 * @brief apollo::canbus::can
 */
namespace apollo {
namespace canbus {
namespace can {

/**
 * @class PCIECanClient
 * @brief The class which defines a PCIE CAN client which inherits CanClient.
 */
class PCIECanClient : public CanClient {
 public:
  /**
   * @brief Initialize the PCIE CAN client by specified CAN card parameters.
   * @param parameter CAN card parameters to initialize the CAN client.
   * @return If the initialization is successful.
   */
  bool Init(const CANCardParameter& parameter) override;

  /**
   * @brief Destructor
   */
  virtual ~PCIECanClient() = default;

  /**
   * @brief Start the PCIE CAN client.
   * @return The status of the start action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Start() override;

  /**
   * @brief Stop the PCIE CAN client.
   */
  void Stop() override;

  /**
   * @brief Send messages
   * @param frames The messages to send.
   * @param frame_num The amount of messages to send.
   * @return The status of the sending action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Send(const std::vector<CanFrame>& frames,
                                 int32_t* const frame_num) override;

  /**
   * @brief Receive messages
   * @param frames The messages to receive.
   * @param frame_num The amount of messages to receive.
   * @return The status of the receiving action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Receive(std::vector<CanFrame>* const frames,
                                    int32_t* const frame_num) override;

  /**
   * @brief Get the error string.
   * @param status The status to get the error string.
   */
  std::string GetErrorString(const int32_t status) override;

 private:
};

}  // namespace can
}  // namespace canbus
}  // namespace apollo
