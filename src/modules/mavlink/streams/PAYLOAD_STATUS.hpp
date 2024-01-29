/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef PAYLOAD_STATUS_HPP
#define PAYLOAD_STATUS_HPP

#include <uORB/topics/payload_status.h>
#include <cstdio>
#include <iostream>

class MavlinkStreamPayloadStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamPayloadStatus(mavlink); }

	static constexpr const char *get_name_static() { return "PAYLOAD_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_PAYLOAD_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _payload_status_sub.advertised() ? MAVLINK_MSG_ID_PAYLOAD_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamPayloadStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _payload_status_sub{ORB_ID(payload_status)};

	bool send() override
	{
		payload_status_s payload_status;

		if (_payload_status_sub.update(&payload_status)) {

			mavlink_payload_status_t msg{};

			msg.pos_x = payload_status.pos_x;
			msg.pos_y = payload_status.pos_y;
			msg.pos_z = payload_status.pos_z;
			msg.linear_velocity_x = payload_status.linear_velocity_x;
			msg.linear_velocity_y = payload_status.linear_velocity_y;
			msg.linear_velocity_z = payload_status.linear_velocity_z;
			msg.angular_velocity_x = payload_status.angular_velocity_x;
			msg.angular_velocity_y = payload_status.angular_velocity_y;
			msg.angular_velocity_z = payload_status.angular_velocity_z;
			memcpy(msg.payload_additional_info, payload_status.payload_additional_info, sizeof(payload_status.payload_additional_info));


			mavlink_msg_payload_status_send_struct(_mavlink->get_channel(), &msg);
			std::cout <<"uorb => mavlink - message was sent !!!!"<< std::endl;
			return true;
		}

		return false;
	}
};

#endif // BATTERY_STATUS_HPP
