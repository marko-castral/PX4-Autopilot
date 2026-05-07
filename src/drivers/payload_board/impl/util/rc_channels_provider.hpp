#pragma once
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <cstdint>
#include <uORB/Subscription.hpp>

namespace payload_board
{
class RcChannelsProvider
{
public:
	RcChannelsProvider(uORB::Subscription &input_rc_sub, uORB::Subscription &manual_control_sub, uint8_t aux_number, uint8_t channel)
		: _input_rc_sub{input_rc_sub}, _manual_control_sub{manual_control_sub}, _aux_number{aux_number}, _channel{channel}
	{
		static_assert(sizeof(input_rc_s::values) >= kDataLength,
			      "input_rc_s::values array must be at least the size of 16 "
			      "channels times 2 bytes");
	}

	bool get_rc_channels(uint16_t *channels) const
	{
		bool channels_updated{false};

		if (_manual_control_sub.updated()) {
			manual_control_setpoint_s manual_control_setpoint{};
			_manual_control_sub.copy(&manual_control_setpoint);

			if (manual_control_setpoint.valid) {
				if (manual_control_setpoint.data_source == manual_control_setpoint_s::SOURCE_RC) {
					if (_input_rc_sub.updated()) {
						input_rc_s input_rc{};
						_input_rc_sub.copy(&input_rc);
						memcpy(channels, input_rc.values, sizeof(input_rc.values));
						channels_updated = !input_rc.rc_lost;
					}

				} else {
					float aux_value = 0.0f;

					switch (_aux_number) {
					case 0:
						aux_value = manual_control_setpoint.aux1;
						break;

					case 1:
						aux_value = manual_control_setpoint.aux2;
						break;

					case 2:
						aux_value = manual_control_setpoint.aux3;
						break;

					case 3:
						aux_value = manual_control_setpoint.aux4;
						break;

					case 4:
						aux_value = manual_control_setpoint.aux5;
						break;

					case 5:
						aux_value = manual_control_setpoint.aux6;
						break;

					default:
						break;
					}

					channels[_channel] = static_cast<uint16_t>(
								     ((aux_value + 1.f) * 500.f) + 1000.f); // map [-1, 1] to [0, 2000]
					channels_updated = true;
				}
			}
		}

		return channels_updated;
	}

private:
	static constexpr const uint8_t kRcChannelsCount{16};
	static constexpr const size_t kDataLength = kRcChannelsCount * sizeof(uint16_t);

	uORB::Subscription &_input_rc_sub;
	uORB::Subscription &_manual_control_sub;
	uint8_t _aux_number;
	uint8_t _channel;
};
}  // namespace payload_board
