/*
 * ErrorOrPrintMessage.h
 *
 *  Created on: Mar 1, 2019
 *      Author: slibert
 *
 * Encapsulates the definition of a WPI Logging Error/Warning or Print Message,
 * which is sent from the robot's Logging Server to one or more clients.
 */

#ifndef ERRORORPRINTMESSAGE_H_
#define ERRORORPRINTMESSAGE_H_

#include <stdint.h>
#include <arpa/inet.h>

#define WPI_LOG_MSG_TYPE_ERROR_OR_WARNING	11
#define WPI_LOG_MSG_TYPE_PRINT              12  // Console (STDOUT) message

struct LengthString {
	uint16_t len;
	char chars[1];  // Actual length depends upon Len

	LengthString(uint16_t len);
	static int GetTotalSize(uint16_t string_len) { return string_len + sizeof(uint16_t); }
} __attribute__((packed));

struct ByteLengthString {
	uint8_t len;
	char chars[1];  // Actual length depends upon Len

	ByteLengthString(uint8_t len);
	static int GetTotalSize(uint8_t string_len) { return string_len + sizeof(uint8_t); }
} __attribute__((packed));

struct ErrorWarningMessageHeader {
	uint16_t     numOccur;	// Number of successive occurrences (>1 if messages are suppressed)
	int32_t      errorCode;
	uint8_t      flags;
} __attribute((packed));

struct MessageHeader {
	MessageHeader *next;
	uint16_t       len;			// Length of message
	uint8_t        msg_type;	// Type of message
	float          timeStamp;	// num secs since user code start // TODO:  Update code that calculates this?
	short          seqNumber;
} __attribute__((packed));

struct RobotVersionDataHeader {
	uint16_t		len;		// Length of tag contents (not including this length variable itself)
	uint8_t			msg_type;	// message type for version data goes here
	uint32_t		id;			// ID of model, 0 otherwise
	// LengthString element name/model
	// LengthString version string
} __attribute__((packed));

class ErrorOrPrintMessage {

protected:
	int FormatMessageHeader(uint8_t msg_type, unsigned char *output_buffer, int max_len, int sequence_number) {
		if (max_len < static_cast<int>(sizeof(MessageHeader))) return -1;

		MessageHeader *header = (MessageHeader *)output_buffer;
		header->next = 0;
		header->msg_type = msg_type;
		header->timeStamp = 0.0; // TODO:  Get Current timestamp as floating point number
		header->seqNumber = htons(sequence_number);

		return sizeof(MessageHeader);
	}

public:

	ErrorOrPrintMessage() {}

	static inline float NetworkOrderedU32ToFloat(uint32_t net_ordered_float) {
		uint32_t host_ordered_float = ntohl(net_ordered_float);
		float ret;
		memcpy(&ret, &host_ordered_float, sizeof(float));
		return ret;
	}

	static inline uint32_t FloatToNetworkOrderedU32(float value) {
		uint32_t host_ordered_float;
		memcpy(&host_ordered_float, &value, sizeof(uint32_t));
		return htonl(host_ordered_float);
	}

	// Formatter for VersionData Messages
	// If outputbuffer is NULL, calculates required length.
	// If outputbuffer is ! NULL, populates output buffer with message content.
	// In both cases, returns the valid message length, or -1 of that length exceeds max_len.
	int FormatVersionDataMessage(unsigned char *output_buffer, int max_len, uint32_t id, uint8_t msg_type, size_t num_element, const char *element_name[], const char *version_string[])
	{
		int running_len = sizeof(RobotVersionDataHeader);
		if (output_buffer) {
			if (max_len < static_cast<int>(sizeof(RobotVersionDataHeader))) {
				return -1;
			}
			RobotVersionDataHeader *header = (RobotVersionDataHeader *)output_buffer;
			header->id = id; // TODO:  Is there ever non-zero:?
			// header->len is filled in later
			header->msg_type = msg_type;
		}
		if (running_len > max_len) return -1;

		for (size_t i = 0; i < num_element; i++) {
			size_t element_name_len = (element_name[i] != 0) ? strlen(element_name[i]) : 0;
			if (output_buffer && (element_name_len > 0)) {
				ByteLengthString * element_name_string = (ByteLengthString *)(output_buffer + running_len);
				element_name_string->len = element_name_len;
				memcpy(element_name_string->chars, element_name[i], element_name_len);
			}
			running_len += ByteLengthString::GetTotalSize(element_name_len);
			if (running_len > max_len) return -1;

			size_t version_string_len = (version_string[i] != 0) ? strlen(version_string[i]) : 0;
			if (output_buffer && (version_string_len > 0)) {
				ByteLengthString * version_string_string = (ByteLengthString *)(output_buffer + running_len);
				version_string_string->len = version_string_len;
				memcpy(version_string_string->chars, version_string[i], version_string_len);
			}
			running_len += ByteLengthString::GetTotalSize(version_string_len);
			if (running_len > max_len) return -1;

			// If the special "end of the list" entry, append an extra null.
			if ((element_name_len == 0) && (version_string_len ==0)) {
				output_buffer[running_len] = '\0';
				running_len += 1;
			}
		}

		if (output_buffer) {
			RobotVersionDataHeader *header = (RobotVersionDataHeader *)output_buffer;
			// length in header does not include the length value.
			uint16_t actual_len = running_len - sizeof(uint16_t);
			header->len = htons(actual_len);
		}


		return running_len;
	}


	// Constructor for Print (e.g., Console) Messages
	// If outputbuffer is NULL, calculates required length.
	// If outputbuffer is ! NULL, populates output buffer with message content.
	// In both cases, returns the valid message length, or -1 of that length exceeds max_len.
	int FormatPrintMessage(unsigned char *output_buffer, int max_len, int sequence_number, char *chars)
	{
		if (!chars) return -1;

		size_t string_length = strlen(chars);
		if (!output_buffer) {
			int len = sizeof(MessageHeader) + LengthString::GetTotalSize(string_length);
			return (len > max_len) ? -1 : len;
		}

		int msg_header_len = FormatMessageHeader(WPI_LOG_MSG_TYPE_PRINT, output_buffer, max_len, sequence_number);
		if (msg_header_len < 0) return -1;

		if ((msg_header_len + static_cast<int>(string_length)) > max_len) return -1;

		char *print_message = (char *)(output_buffer + msg_header_len);
		memcpy(print_message, chars, string_length);

		int total_len = msg_header_len + string_length;

		if (output_buffer) {
			MessageHeader *header = (MessageHeader *)output_buffer;
			// length in header does not include the link * and the length value.
			uint16_t actual_len = total_len - (sizeof(MessageHeader *) + sizeof(uint16_t));
			header->len = htons(actual_len);
		}
		return total_len;
	}
	// Constructor for Error/Warning Messages
	// If outputbuffer is NULL, calculates required length.
	// If outputbuffer is ! NULL, populates output buffer with message content.
	// In both cases, returns the valid message length, or -1 of that length exceeds max_len.
	int FormatErrorMessage(unsigned char *output_buffer, int max_len, int sequence_number, uint16_t numOccur, int32_t errorCode, uint8_t flags, const char *details, const char *location, const char *callStack)
	{
		int running_len;
		if (!output_buffer) {
			running_len = sizeof(MessageHeader);
		} else {
			running_len = FormatMessageHeader(WPI_LOG_MSG_TYPE_ERROR_OR_WARNING, output_buffer, max_len, sequence_number);
			if (running_len < 0) return -1;
		}
		if (running_len > max_len) return -1;

		if (output_buffer) {
			ErrorWarningMessageHeader * error_header = (ErrorWarningMessageHeader *)(output_buffer + running_len);
			error_header->numOccur = htons(numOccur);
			error_header->errorCode = htonl(errorCode);
			error_header->flags = flags;
		}
		running_len += sizeof(ErrorWarningMessageHeader);
		if (running_len > max_len) return -1;

		size_t detail_len = strlen(details);
		if (output_buffer) {
			LengthString * detail_string = (LengthString *)(output_buffer + running_len);
			detail_string->len = htons(detail_len);
			memcpy(detail_string->chars, details, detail_len);
		}
		running_len += LengthString::GetTotalSize(detail_len);
		if (running_len > max_len) return -1;

		size_t location_len = strlen(location);
		if (output_buffer) {
			LengthString * location_string = (LengthString *)(output_buffer + running_len);
			location_string->len = htons(location_len);
			memcpy(location_string->chars, location, location_len);
		}
		running_len += LengthString::GetTotalSize(location_len);
		if (running_len > max_len) return -1;

		size_t callStack_len = strlen(callStack);
		if (output_buffer) {
			LengthString * callStack_string = (LengthString *)(output_buffer + running_len);
			callStack_string->len = htons(callStack_len);
			memcpy(callStack_string->chars, callStack, callStack_len);
		}
		running_len += LengthString::GetTotalSize(callStack_len);
		if (running_len > max_len) return -1;

		if (output_buffer) {
			MessageHeader *header = (MessageHeader *)output_buffer;
			// length in header does not include the link * and the length value.
			header->len = htons(running_len - (sizeof(MessageHeader *) + sizeof(uint16_t)));
		}

		return running_len;
	}
	~ErrorOrPrintMessage() {}
};

#endif /* ERRORORPRINTMESSAGE_H_ */
