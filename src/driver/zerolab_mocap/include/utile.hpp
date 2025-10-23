
// Function to decode byte array into an array of floats
std::vector<float> decodeFloatByteArray(const std::vector<uint8_t>& byteArray) {
  std::vector<float> decodedValues;
  size_t numFloats = byteArray.size() / sizeof(float);  // Corrected to 4 bytes per float
  for (size_t i = 0; i < numFloats; ++i) {
    float value;
    std::memcpy(&value, &byteArray[i * sizeof(float)], sizeof(float));  // Read 4 bytes
    decodedValues.push_back(value);
  }
  return decodedValues;
}

std::vector<uint16_t> bytesToUInt16Vector(const std::vector<uint8_t>& bytes) {
  std::vector<uint16_t> result;

  for (size_t i = 0; i < bytes.size(); i += 2) {
    uint16_t value = (static_cast<uint16_t>(bytes[i + 1]) << 8) | bytes[i];
    result.push_back(value);
  }

  return result;
}
