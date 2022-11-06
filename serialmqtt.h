#pragma once

#include <Arduino.h>

namespace SerialMQTT {
	// The function that will be called upon receipt of topic & payload
	void SetTopicHandler(void (*f)(String topic, String payload));
	// Process Serial input and call handler if \n is reached
	void Update();
	// Publish a payload to a topic via SerialMQTT
	void Publish(String topic, String payload);
};