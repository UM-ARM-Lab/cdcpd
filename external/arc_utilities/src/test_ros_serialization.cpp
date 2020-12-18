<<<<<<< HEAD
#include <visualization_msgs/Marker.h>

#include <random>

#include "arc_utilities/pretty_print.hpp"
#include "arc_utilities/serialization.hpp"
#include "arc_utilities/serialization_eigen.hpp"
#include "arc_utilities/serialization_ros.hpp"

using namespace arc_utilities;

template <typename T>
void TestFixedSizePOD(const T& pod) {
  auto buffer = std::vector<uint8_t>();
  auto bytes_written = SerializeFixedSizePOD(pod, buffer);
  auto deserialized = DeserializeFixedSizePOD<T>(buffer, 0);
  if (bytes_written != deserialized.second) {
    std::cerr << "Bytes written does not match bytes read " << bytes_written << " " << deserialized.second;
    assert(bytes_written == deserialized.second);
  }
  if (pod != deserialized.first) {
    std::cerr << "Pre serialized data does not match deserialized data " << pod << " " << deserialized.first
              << std::endl;
    assert(pod == deserialized.first);
  }
}

template <typename T>
void TestFixedSizeFloatPOD(std::mt19937_64& generator) {
  auto rand = std::uniform_real_distribution<T>(-std::numeric_limits<T>::max(), std::numeric_limits<T>::max());
  auto test = rand(generator);
  TestFixedSizePOD<T>(test);
}

template <typename T>
void TestFixedSizeIntPOD(std::mt19937_64& generator) {
  auto rand = std::uniform_int_distribution<T>(std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
  auto test = rand(generator);
  TestFixedSizePOD<T>(test);
}

// Assumes that the vector is a POD type
template <typename T>
void TestVector(const std::vector<T>& vec) {
  auto buffer = std::vector<uint8_t>();
  auto bytes_written = SerializeVector<T>(vec, buffer, SerializeFixedSizePOD<T>);
  auto deserialized = DeserializeVector<T>(buffer, 0, DeserializeFixedSizePOD<T>);
  if (bytes_written != deserialized.second) {
    std::cerr << "Bytes written does not match bytes read " << bytes_written << " " << deserialized.second;
    assert(bytes_written == deserialized.second);
  }
  if (vec != deserialized.first) {
    std::cerr << "Pre serialized data does not match deserialized data " << PrettyPrint::PrettyPrint(vec, true, " ")
              << "    " << PrettyPrint::PrettyPrint(deserialized.first, true, " ") << std::endl;
    assert(vec == deserialized.first);
  }
}

template <typename T>
void TestIntVector(std::mt19937_64& generator) {
  auto rand_length = std::uniform_int_distribution<int>(10, 20);
  auto rand = std::uniform_int_distribution<T>(std::numeric_limits<T>::min(), std::numeric_limits<T>::max());
  std::vector<T> vec(rand_length(generator));
  for (auto& data : vec) {
    data = rand(generator);
  }
  TestVector<T>(vec);
}

template <typename T>
void TestFloatVector(std::mt19937_64& generator) {
  auto rand_length = std::uniform_int_distribution<int>(10, 20);
  auto rand = std::uniform_real_distribution<T>(-std::numeric_limits<T>::max(), std::numeric_limits<T>::max());
  std::vector<T> vec(rand_length(generator));
  for (auto& data : vec) {
    data = rand(generator);
  }
  TestVector<T>(vec);
}

// Assumes POD keys and values - tests pair in the process
template <typename Key, typename Value>
void TestMap(const std::map<Key, Value>& map) {
  auto buffer = std::vector<uint8_t>();
  auto bytes_written = SerializeMap<Key, Value>(map, buffer, SerializeFixedSizePOD<Key>, SerializeFixedSizePOD<Value>);
  auto deserialized =
      DeserializeMap<Key, Value>(buffer, 0, DeserializeFixedSizePOD<Key>, DeserializeFixedSizePOD<Value>);
  if (bytes_written != deserialized.second) {
    std::cerr << "Bytes written does not match bytes read " << bytes_written << " " << deserialized.second;
    assert(bytes_written == deserialized.second);
  }
  if (map != deserialized.first) {
    std::cerr << "Pre serialized data does not match deserialized data " << PrettyPrint::PrettyPrint(map, true, " ")
              << "    " << PrettyPrint::PrettyPrint(deserialized.first, true, " ") << std::endl;
    assert(map == deserialized.first);
  }
}

template <typename Key, typename Value>
void TestIntFloatMap(std::mt19937_64& generator) {
  auto rand_length = std::uniform_int_distribution<int>(10, 20);
  auto rand_int = std::uniform_int_distribution<Key>(std::numeric_limits<Key>::min(), std::numeric_limits<Key>::max());
  auto rand_float =
      std::uniform_real_distribution<Value>(-std::numeric_limits<Value>::max(), std::numeric_limits<Value>::max());
  std::map<Key, Value> map;
  auto length = rand_length(generator);
  for (int i = 0; i < length; ++i) {
    auto key = rand_int(generator);
    auto value = rand_float(generator);
    map[key] = value;
  }
  TestMap<Key, Value>(map);
}

template <typename Key, typename Value>
void TestFloatIntMap(std::mt19937_64& generator) {
  auto rand_length = std::uniform_int_distribution<int>(10, 20);
  auto rand_float =
      std::uniform_real_distribution<Key>(-std::numeric_limits<Key>::max(), std::numeric_limits<Key>::max());
  auto rand_int =
      std::uniform_int_distribution<Value>(std::numeric_limits<Value>::min(), std::numeric_limits<Value>::max());
  std::map<Key, Value> map;
  auto length = rand_length(generator);
  for (int i = 0; i < length; ++i) {
    auto key = rand_float(generator);
    auto value = rand_int(generator);
    map[key] = value;
  }
  TestMap<Key, Value>(map);
}

std::string RandomString(std::mt19937_64& generator) {
  static const char alphanum[] =
      "0123456789_"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz";

  auto rand_length = std::uniform_int_distribution<uint32_t>(0, 20);
  auto rand_char = std::uniform_int_distribution<uint32_t>(0, sizeof(alphanum) - 1);
  std::string str;
  auto length = rand_length(generator);
  while (str.size() < length) {
    str += alphanum[rand_char(generator)];
  }
  return str;
}

std_msgs::Header RandomHeader(std::mt19937_64& generator) {
  std_msgs::Header header;
  auto rand_uint32_t = std::uniform_int_distribution<uint32_t>(0, std::numeric_limits<uint32_t>::max());
  header.seq = rand_uint32_t(generator);
  header.stamp.sec = rand_uint32_t(generator);
  header.stamp.nsec = rand_uint32_t(generator);
  header.frame_id = RandomString(generator);
  return header;
}

geometry_msgs::Point RandomPoint(std::mt19937_64& generator) {
  auto rand_float =
      std::uniform_real_distribution<double>(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  geometry_msgs::Point p;
  p.x = rand_float(generator);
  p.y = rand_float(generator);
  p.z = rand_float(generator);
  return p;
}

geometry_msgs::Vector3 RandomVector3(std::mt19937_64& generator) {
  auto rand_float =
      std::uniform_real_distribution<double>(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  geometry_msgs::Vector3 v;
  v.x = rand_float(generator);
  v.y = rand_float(generator);
  v.z = rand_float(generator);
  return v;
}

geometry_msgs::Quaternion RandomQuaternion(std::mt19937_64& generator) {
  auto rand_float =
      std::uniform_real_distribution<double>(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  geometry_msgs::Quaternion q;
  q.x = rand_float(generator);
  q.y = rand_float(generator);
  q.z = rand_float(generator);
  q.w = rand_float(generator);
  return q;
}

geometry_msgs::Pose RandomPose(std::mt19937_64& generator) {
  geometry_msgs::Pose p;
  p.position = RandomPoint(generator);
  p.orientation = RandomQuaternion(generator);
  return p;
}

geometry_msgs::Transform RandomTransform(std::mt19937_64& generator) {
  geometry_msgs::Transform t;
  t.translation = RandomVector3(generator);
  t.rotation = RandomQuaternion(generator);
  return t;
}

std_msgs::ColorRGBA RandomColorRGBA(std::mt19937_64& generator) {
  auto rand_float = std::uniform_real_distribution<float>(0, 1);
  std_msgs::ColorRGBA c;
  c.r = rand_float(generator);
  c.g = rand_float(generator);
  c.b = rand_float(generator);
  c.a = rand_float(generator);
  return c;
}

std::vector<geometry_msgs::Point> RandomPointsVector(std::mt19937_64& generator) {
  auto rand_length = std::uniform_int_distribution<int>(10, 20);
  auto vec = std::vector<geometry_msgs::Point>(rand_length(generator));
  for (auto& point : vec) {
    point = RandomPoint(generator);
  }
  return vec;
}

std::vector<std_msgs::ColorRGBA> RandomColorRGBAVector(std::mt19937_64& generator) {
  auto rand_length = std::uniform_int_distribution<int>(10, 20);
  auto vec = std::vector<std_msgs::ColorRGBA>(rand_length(generator));
  for (auto& color : vec) {
    color = RandomColorRGBA(generator);
  }
  return vec;
}

visualization_msgs::Marker RandomMarker(std::mt19937_64& generator) {
  visualization_msgs::Marker m;
  m.header = RandomHeader(generator);
  m.ns = RandomString(generator);
  m.id = std::uniform_int_distribution<int32_t>(1, std::numeric_limits<int32_t>::max())(generator);
  m.action = std::uniform_int_distribution<int32_t>(m.ADD, m.DELETEALL)(generator);
  m.pose = RandomPose(generator);
  m.scale = RandomVector3(generator);
  m.color = RandomColorRGBA(generator);
  m.lifetime = ros::Duration(std::uniform_real_distribution<double>(0, 10)(generator));
  m.frame_locked = false;
  m.points = RandomPointsVector(generator);
  m.colors = RandomColorRGBAVector(generator);
  m.text = RandomString(generator);
  m.mesh_resource = RandomString(generator);
  m.mesh_use_embedded_materials = false;
  return m;
}

bool operator==(const std_msgs::Header& lhs, const std_msgs::Header& rhs) {
  return (lhs.seq == rhs.seq && lhs.stamp == rhs.stamp && lhs.frame_id == rhs.frame_id);
}

bool operator==(const geometry_msgs::Vector3& lhs, const geometry_msgs::Vector3& rhs) {
  return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

bool operator==(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
  return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

bool operator==(const geometry_msgs::Quaternion& lhs, const geometry_msgs::Quaternion& rhs) {
  return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w);
}

bool operator==(const geometry_msgs::Pose& lhs, const geometry_msgs::Pose& rhs) {
  return (lhs.position == rhs.position && lhs.orientation == rhs.orientation);
}

bool operator==(const geometry_msgs::Transform& lhs, const geometry_msgs::Transform& rhs) {
  return (lhs.translation == rhs.translation && lhs.rotation == rhs.rotation);
}

bool operator==(const geometry_msgs::PoseStamped& lhs, const geometry_msgs::PoseStamped& rhs) {
  return (lhs.header == rhs.header && lhs.pose == rhs.pose);
}

bool operator==(const geometry_msgs::TransformStamped& lhs, const geometry_msgs::TransformStamped& rhs) {
  return (lhs.header == rhs.header && lhs.transform == rhs.transform && lhs.child_frame_id == rhs.child_frame_id);
}

bool operator==(const std_msgs::ColorRGBA& lhs, const std_msgs::ColorRGBA& rhs) {
  return (lhs.r == rhs.r && lhs.g == rhs.g && lhs.b == rhs.b && lhs.a == rhs.a);
=======
#include "arc_utilities/serialization.hpp"
#include "arc_utilities/serialization_eigen.hpp"
#include "arc_utilities/serialization_ros.hpp"
#include "arc_utilities/pretty_print.hpp"
#include <random>
#include <visualization_msgs/Marker.h>

using namespace arc_utilities;

template<typename T>
void TestFixedSizePOD(const T& pod)
{
    auto buffer = std::vector<uint8_t>();
    auto bytes_written = SerializeFixedSizePOD(pod, buffer);
    auto deserialized = DeserializeFixedSizePOD<T>(buffer, 0);
    if (bytes_written != deserialized.second)
    {
        std::cerr << "Bytes written does not match bytes read "
                  << bytes_written << " " << deserialized.second;
        assert(bytes_written == deserialized.second);
    }
    if (pod != deserialized.first)
    {
        std::cerr << "Pre serialized data does not match deserialized data "
                  << pod << " " << deserialized.first
                  << std::endl;
        assert(pod == deserialized.first);
    }
}

template<typename T>
void TestFixedSizeFloatPOD(std::mt19937_64& generator)
{
    auto rand = std::uniform_real_distribution<T>(
                -std::numeric_limits<T>::max(),
                std::numeric_limits<T>::max());
    auto test = rand(generator);
    TestFixedSizePOD<T>(test);
}

template<typename T>
void TestFixedSizeIntPOD(std::mt19937_64& generator)
{
    auto rand = std::uniform_int_distribution<T>(
                std::numeric_limits<T>::min(),
                std::numeric_limits<T>::max());
    auto test = rand(generator);
    TestFixedSizePOD<T>(test);
}

// Assumes that the vector is a POD type
template<typename T>
void TestVector(const std::vector<T>& vec)
{
    auto buffer = std::vector<uint8_t>();
    auto bytes_written = SerializeVector<T>(vec, buffer, SerializeFixedSizePOD<T>);
    auto deserialized = DeserializeVector<T>(buffer, 0, DeserializeFixedSizePOD<T>);
    if (bytes_written != deserialized.second)
    {
        std::cerr << "Bytes written does not match bytes read "
                  << bytes_written << " " << deserialized.second;
        assert(bytes_written == deserialized.second);
    }
    if (vec != deserialized.first)
    {
        std::cerr << "Pre serialized data does not match deserialized data "
                  << PrettyPrint::PrettyPrint(vec, true, " ") << "    "
                  << PrettyPrint::PrettyPrint(deserialized.first, true, " ")
                  << std::endl;
        assert(vec == deserialized.first);
    }
}

template<typename T>
void TestIntVector(std::mt19937_64& generator)
{
    auto rand_length = std::uniform_int_distribution<int>(10, 20);
    auto rand = std::uniform_int_distribution<T>(
                std::numeric_limits<T>::min(),
                std::numeric_limits<T>::max());
    std::vector<T> vec(rand_length(generator));
    for (auto& data : vec)
    {
        data = rand(generator);
    }
    TestVector<T>(vec);
}

template<typename T>
void TestFloatVector(std::mt19937_64& generator)
{
    auto rand_length = std::uniform_int_distribution<int>(10, 20);
    auto rand = std::uniform_real_distribution<T>(
                -std::numeric_limits<T>::max(),
                std::numeric_limits<T>::max());
    std::vector<T> vec(rand_length(generator));
    for (auto& data : vec)
    {
        data = rand(generator);
    }
    TestVector<T>(vec);
}

// Assumes POD keys and values - tests pair in the process
template<typename Key, typename Value>
void TestMap(const std::map<Key, Value>& map)
{
    auto buffer = std::vector<uint8_t>();
    auto bytes_written = SerializeMap<Key, Value>(map, buffer, SerializeFixedSizePOD<Key>, SerializeFixedSizePOD<Value>);
    auto deserialized = DeserializeMap<Key, Value>(buffer, 0, DeserializeFixedSizePOD<Key>, DeserializeFixedSizePOD<Value>);
    if (bytes_written != deserialized.second)
    {
        std::cerr << "Bytes written does not match bytes read "
                  << bytes_written << " " << deserialized.second;
        assert(bytes_written == deserialized.second);
    }
    if (map != deserialized.first)
    {
        std::cerr << "Pre serialized data does not match deserialized data "
                  << PrettyPrint::PrettyPrint(map, true, " ") << "    "
                  << PrettyPrint::PrettyPrint(deserialized.first, true, " ")
                  << std::endl;
        assert(map == deserialized.first);
    }
}

template<typename Key, typename Value>
void TestIntFloatMap(std::mt19937_64& generator)
{
    auto rand_length = std::uniform_int_distribution<int>(10, 20);
    auto rand_int = std::uniform_int_distribution<Key>(
                std::numeric_limits<Key>::min(),
                std::numeric_limits<Key>::max());
    auto rand_float = std::uniform_real_distribution<Value>(
                -std::numeric_limits<Value>::max(),
                std::numeric_limits<Value>::max());
    std::map<Key, Value> map;
    auto length = rand_length(generator);
    for (int i = 0; i < length; ++i)
    {
        auto key = rand_int(generator);
        auto value = rand_float(generator);
        map[key] = value;
    }
    TestMap<Key, Value>(map);
}

template<typename Key, typename Value>
void TestFloatIntMap(std::mt19937_64& generator)
{
    auto rand_length = std::uniform_int_distribution<int>(10, 20);
    auto rand_float = std::uniform_real_distribution<Key>(
                -std::numeric_limits<Key>::max(),
                std::numeric_limits<Key>::max());
    auto rand_int = std::uniform_int_distribution<Value>(
                std::numeric_limits<Value>::min(),
                std::numeric_limits<Value>::max());
    std::map<Key, Value> map;
    auto length = rand_length(generator);
    for (int i = 0; i < length; ++i)
    {
        auto key = rand_float(generator);
        auto value = rand_int(generator);
        map[key] = value;
    }
    TestMap<Key, Value>(map);
}

std::string RandomString(std::mt19937_64& generator)
{
    static const char alphanum[] =
            "0123456789_"
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz";

    auto rand_length = std::uniform_int_distribution<uint32_t>(0, 20);
    auto rand_char = std::uniform_int_distribution<uint32_t>(0, sizeof(alphanum) - 1);
    std::string str;
    auto length = rand_length(generator);
    while (str.size() < length)
    {
        str += alphanum[rand_char(generator)];
    }
    return str;
}

std_msgs::Header RandomHeader(std::mt19937_64& generator)
{
    std_msgs::Header header;
    auto rand_uint32_t = std::uniform_int_distribution<uint32_t>(
                0, std::numeric_limits<uint32_t>::max());
    header.seq = rand_uint32_t(generator);
    header.stamp.sec = rand_uint32_t(generator);
    header.stamp.nsec = rand_uint32_t(generator);
    header.frame_id = RandomString(generator);
    return header;
}

geometry_msgs::Point RandomPoint(std::mt19937_64& generator)
{
    auto rand_float = std::uniform_real_distribution<double>(
                -std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max());
    geometry_msgs::Point p;
    p.x = rand_float(generator);
    p.y = rand_float(generator);
    p.z = rand_float(generator);
    return p;
}

geometry_msgs::Vector3 RandomVector3(std::mt19937_64& generator)
{
    auto rand_float = std::uniform_real_distribution<double>(
                -std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max());
    geometry_msgs::Vector3 v;
    v.x = rand_float(generator);
    v.y = rand_float(generator);
    v.z = rand_float(generator);
    return v;
}

geometry_msgs::Quaternion RandomQuaternion(std::mt19937_64& generator)
{
    auto rand_float = std::uniform_real_distribution<double>(
                -std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max());
    geometry_msgs::Quaternion q;
    q.x = rand_float(generator);
    q.y = rand_float(generator);
    q.z = rand_float(generator);
    q.w = rand_float(generator);
    return q;
}

geometry_msgs::Pose RandomPose(std::mt19937_64& generator)
{
    geometry_msgs::Pose p;
    p.position = RandomPoint(generator);
    p.orientation = RandomQuaternion(generator);
    return p;
}

geometry_msgs::Transform RandomTransform(std::mt19937_64& generator)
{
    geometry_msgs::Transform t;
    t.translation = RandomVector3(generator);
    t.rotation = RandomQuaternion(generator);
    return t;
}

std_msgs::ColorRGBA RandomColorRGBA(std::mt19937_64& generator)
{
    auto rand_float = std::uniform_real_distribution<float>(0, 1);
    std_msgs::ColorRGBA c;
    c.r = rand_float(generator);
    c.g = rand_float(generator);
    c.b = rand_float(generator);
    c.a = rand_float(generator);
    return c;
}

std::vector<geometry_msgs::Point> RandomPointsVector(std::mt19937_64& generator)
{
    auto rand_length = std::uniform_int_distribution<int>(10, 20);
    auto vec = std::vector<geometry_msgs::Point>(rand_length(generator));
    for (auto& point : vec)
    {
        point = RandomPoint(generator);
    }
    return vec;
}

std::vector<std_msgs::ColorRGBA> RandomColorRGBAVector(std::mt19937_64& generator)
{
    auto rand_length = std::uniform_int_distribution<int>(10, 20);
    auto vec = std::vector<std_msgs::ColorRGBA>(rand_length(generator));
    for (auto& color : vec)
    {
        color = RandomColorRGBA(generator);
    }
    return vec;
}

visualization_msgs::Marker RandomMarker(std::mt19937_64& generator)
{
    visualization_msgs::Marker m;
    m.header = RandomHeader(generator);
    m.ns = RandomString(generator);
    m.id = std::uniform_int_distribution<int32_t>(1, std::numeric_limits<int32_t>::max())(generator);
    m.action = std::uniform_int_distribution<int32_t>(m.ADD, m.DELETEALL)(generator);
    m.pose = RandomPose(generator);
    m.scale = RandomVector3(generator);
    m.color = RandomColorRGBA(generator);
    m.lifetime = ros::Duration(std::uniform_real_distribution<double>(0, 10)(generator));
    m.frame_locked = false;
    m.points = RandomPointsVector(generator);
    m.colors = RandomColorRGBAVector(generator);
    m.text = RandomString(generator);
    m.mesh_resource = RandomString(generator);
    m.mesh_use_embedded_materials = false;
    return m;
}

bool operator==(const std_msgs::Header& lhs, const std_msgs::Header& rhs)
{
    return (lhs.seq == rhs.seq &&
            lhs.stamp == rhs.stamp &&
            lhs.frame_id == rhs.frame_id);
}

bool operator==(const geometry_msgs::Vector3 &lhs, const geometry_msgs::Vector3 &rhs)
{
    return (lhs.x == rhs.x &&
            lhs.y == rhs.y &&
            lhs.z == rhs.z);
}

bool operator==(const geometry_msgs::Point &lhs, const geometry_msgs::Point &rhs)
{
    return (lhs.x == rhs.x &&
            lhs.y == rhs.y &&
            lhs.z == rhs.z);
}

bool operator==(const geometry_msgs::Quaternion &lhs, const geometry_msgs::Quaternion &rhs)
{
    return (lhs.x == rhs.x &&
            lhs.y == rhs.y &&
            lhs.z == rhs.z &&
            lhs.w == rhs.w);
}

bool operator==(const geometry_msgs::Pose &lhs, const geometry_msgs::Pose &rhs)
{
    return (lhs.position == rhs.position &&
            lhs.orientation == rhs.orientation);
}

bool operator==(const geometry_msgs::Transform &lhs, const geometry_msgs::Transform &rhs)
{
    return (lhs.translation == rhs.translation &&
            lhs.rotation == rhs.rotation);
}

bool operator==(const geometry_msgs::PoseStamped& lhs, const geometry_msgs::PoseStamped& rhs)
{
    return (lhs.header == rhs.header &&
            lhs.pose == rhs.pose);
}

bool operator==(const geometry_msgs::TransformStamped& lhs, const geometry_msgs::TransformStamped& rhs)
{
    return (lhs.header == rhs.header &&
            lhs.transform == rhs.transform &&
            lhs.child_frame_id == rhs.child_frame_id);
}

bool operator==(const std_msgs::ColorRGBA& lhs, const std_msgs::ColorRGBA& rhs)
{
    return (lhs.r == rhs.r &&
            lhs.g == rhs.g &&
            lhs.b == rhs.b &&
            lhs.a == rhs.a);
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
}

// Writen to address weird compiler template auto-dedeuction failure
// TODO: figure out why the auto-deduction fails and remove this function
<<<<<<< HEAD
template <typename RosType>
bool Equal(const std::vector<RosType>& lhs, const std::vector<RosType>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (size_t idx = 0; idx < lhs.size(); ++idx) {
    if (!(lhs[idx] == rhs[idx])) {
      return false;
    }
  }
  return true;
}

bool operator==(const visualization_msgs::Marker& lhs, const visualization_msgs::Marker& rhs) {
  return (lhs.header == rhs.header && lhs.ns == rhs.ns && lhs.id == rhs.id && lhs.type == rhs.type &&
          lhs.action == rhs.action && lhs.pose == rhs.pose && lhs.scale == rhs.scale && lhs.color == rhs.color &&
          lhs.lifetime == rhs.lifetime && lhs.frame_locked == rhs.frame_locked && Equal(lhs.points, rhs.points) &&
          Equal(lhs.colors, rhs.colors) && lhs.text == rhs.text && lhs.mesh_resource == rhs.mesh_resource &&
          lhs.mesh_use_embedded_materials == rhs.mesh_use_embedded_materials);
}

// Tests header, pose, point, quaternion
void TestPoseStamped(std::mt19937_64& generator) {
  geometry_msgs::PoseStamped pose;
  pose.header = RandomHeader(generator);
  pose.pose = RandomPose(generator);

  // Manual version
  {
    auto buffer = std::vector<uint8_t>();
    auto bytes_written = SerializePoseStamped(pose, buffer);
    auto deserialized = DeserializePoseStamped(buffer, 0);
    if (bytes_written != deserialized.second) {
      std::cerr << "Bytes written does not match bytes read " << bytes_written << " " << deserialized.second;
      assert(bytes_written == deserialized.second);
    }
    if (!(pose == deserialized.first)) {
      std::cerr << "Pre serialized data does not match deserialized data " << PrettyPrint::PrettyPrint(pose, true, " ")
                << "    " << PrettyPrint::PrettyPrint(deserialized.first, true, " ") << std::endl;
      assert(pose == deserialized.first);
    }
  }
  // Relying on ROS serialization
  {
    auto buffer = std::vector<uint8_t>();
    auto bytes_written = RosMessageSerializationWrapper(pose, buffer);
    auto deserialized = RosMessageDeserializationWrapper<geometry_msgs::PoseStamped>(buffer, 0);
    if (bytes_written != deserialized.second) {
      std::cerr << "Bytes written does not match bytes read " << bytes_written << " " << deserialized.second;
      assert(bytes_written == deserialized.second);
    }
    if (!(pose == deserialized.first)) {
      std::cerr << "Pre serialized data does not match deserialized data " << PrettyPrint::PrettyPrint(pose, true, " ")
                << "    " << PrettyPrint::PrettyPrint(deserialized.first, true, " ") << std::endl;
      assert(pose == deserialized.first);
    }
  }
}

// Tests header, transform, vector3, quaternion, and empty string
void TestTransformStamped(std::mt19937_64& generator) {
  geometry_msgs::TransformStamped transform;
  transform.header = RandomHeader(generator);
  transform.transform = RandomTransform(generator);
  transform.child_frame_id = "";

  // Manual version
  {
    auto buffer = std::vector<uint8_t>();
    auto bytes_written = SerializeTransformStamped(transform, buffer);
    auto deserialized = DeserializeTransformStamped(buffer, 0);
    if (bytes_written != deserialized.second) {
      std::cerr << "Bytes written does not match bytes read " << bytes_written << " " << deserialized.second;
      assert(bytes_written == deserialized.second);
    }
    if (!(transform == deserialized.first)) {
      std::cerr << "Pre serialized data does not match deserialized data "
                << PrettyPrint::PrettyPrint(transform, true, " ") << "    "
                << PrettyPrint::PrettyPrint(deserialized.first, true, " ") << std::endl;
      assert(transform == deserialized.first);
    }
  }
  // Relying on ROS serialization
  {
    auto buffer = std::vector<uint8_t>();
    auto bytes_written = RosMessageSerializationWrapper(transform, buffer);
    auto deserialized = RosMessageDeserializationWrapper<geometry_msgs::TransformStamped>(buffer, 0);
    if (bytes_written != deserialized.second) {
      std::cerr << "Bytes written does not match bytes read " << bytes_written << " " << deserialized.second;
      assert(bytes_written == deserialized.second);
    }
    if (!(transform == deserialized.first)) {
      std::cerr << "Pre serialized data does not match deserialized data "
                << PrettyPrint::PrettyPrint(transform, true, " ") << "    "
                << PrettyPrint::PrettyPrint(deserialized.first, true, " ") << std::endl;
      assert(transform == deserialized.first);
    }
  }
}

// Tests Marker, and all sub types
void TestVisualizationMarker(std::mt19937_64& generator) {
  auto marker = RandomMarker(generator);
  auto buffer = std::vector<uint8_t>();
  auto bytes_written = RosMessageSerializationWrapper(marker, buffer);
  auto deserialized = RosMessageDeserializationWrapper<visualization_msgs::Marker>(buffer, 0);
  if (bytes_written != deserialized.second) {
    std::cerr << "Bytes written does not match bytes read " << bytes_written << " " << deserialized.second;
    assert(bytes_written == deserialized.second);
  }
  if (!(marker == deserialized.first)) {
    std::cerr << "Pre serialized data does not match deserialized data " << PrettyPrint::PrettyPrint(marker, true, " ")
              << "    " << PrettyPrint::PrettyPrint(deserialized.first, true, " ") << std::endl;
    assert(marker == deserialized.first);
  }
}

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  const auto seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::mt19937_64 generator(seed);

  for (int i = 0; i < 100; ++i) {
    TestFixedSizePOD<bool>(false);
    TestFixedSizePOD<bool>(true);
    TestFixedSizeIntPOD<uint8_t>(generator);
    TestFixedSizeIntPOD<uint16_t>(generator);
    TestFixedSizeIntPOD<uint32_t>(generator);
    TestFixedSizeIntPOD<uint64_t>(generator);
    TestFixedSizeIntPOD<int8_t>(generator);
    TestFixedSizeIntPOD<int16_t>(generator);
    TestFixedSizeIntPOD<int32_t>(generator);
    TestFixedSizeIntPOD<int64_t>(generator);
    TestFixedSizeFloatPOD<float>(generator);
    TestFixedSizeFloatPOD<double>(generator);

    TestIntVector<uint8_t>(generator);
    TestIntVector<uint16_t>(generator);
    TestIntVector<uint32_t>(generator);
    TestIntVector<uint64_t>(generator);
    TestIntVector<int8_t>(generator);
    TestIntVector<int16_t>(generator);
    TestIntVector<int32_t>(generator);
    TestIntVector<int64_t>(generator);
    TestFloatVector<float>(generator);
    TestFloatVector<double>(generator);

    TestIntFloatMap<int, float>(generator);
    TestFloatIntMap<double, uint64_t>(generator);

    TestPoseStamped(generator);
    TestTransformStamped(generator);

    TestVisualizationMarker(generator);
  }

  std::cout << "All tests passed" << std::endl;
=======
template<typename RosType>
bool Equal(const std::vector<RosType>& lhs, const std::vector<RosType>& rhs)
{
    if (lhs.size() != rhs.size())
    {
        return false;
    }
    for (size_t idx = 0; idx < lhs.size(); ++idx)
    {
        if (!(lhs[idx] == rhs[idx]))
        {
            return false;
        }
    }
    return true;
}

bool operator==(const visualization_msgs::Marker& lhs, const visualization_msgs::Marker& rhs)
{
    return (lhs.header == rhs.header &&
            lhs.ns == rhs.ns &&
            lhs.id == rhs.id &&
            lhs.type == rhs.type &&
            lhs.action == rhs.action &&
            lhs.pose == rhs.pose &&
            lhs.scale == rhs.scale &&
            lhs.color == rhs.color &&
            lhs.lifetime == rhs.lifetime &&
            lhs.frame_locked == rhs.frame_locked &&
            Equal(lhs.points, rhs.points) &&
            Equal(lhs.colors, rhs.colors) &&
            lhs.text == rhs.text &&
            lhs.mesh_resource == rhs.mesh_resource &&
            lhs.mesh_use_embedded_materials == rhs.mesh_use_embedded_materials);
}

// Tests header, pose, point, quaternion
void TestPoseStamped(std::mt19937_64& generator)
{
    geometry_msgs::PoseStamped pose;
    pose.header = RandomHeader(generator);
    pose.pose = RandomPose(generator);

    // Manual version
    {
        auto buffer = std::vector<uint8_t>();
        auto bytes_written = SerializePoseStamped(pose, buffer);
        auto deserialized = DeserializePoseStamped(buffer, 0);
        if (bytes_written != deserialized.second)
        {
            std::cerr << "Bytes written does not match bytes read "
                      << bytes_written << " " << deserialized.second;
            assert(bytes_written == deserialized.second);
        }
        if (!(pose == deserialized.first))
        {
            std::cerr << "Pre serialized data does not match deserialized data "
                      << PrettyPrint::PrettyPrint(pose, true, " ") << "    "
                      << PrettyPrint::PrettyPrint(deserialized.first, true, " ")
                      << std::endl;
            assert(pose == deserialized.first);
        }

    }
    // Relying on ROS serialization
    {
        auto buffer = std::vector<uint8_t>();
        auto bytes_written = RosMessageSerializationWrapper(pose, buffer);
        auto deserialized = RosMessageDeserializationWrapper<geometry_msgs::PoseStamped>(buffer, 0);
        if (bytes_written != deserialized.second)
        {
            std::cerr << "Bytes written does not match bytes read "
                      << bytes_written << " " << deserialized.second;
            assert(bytes_written == deserialized.second);
        }
        if (!(pose == deserialized.first))
        {
            std::cerr << "Pre serialized data does not match deserialized data "
                      << PrettyPrint::PrettyPrint(pose, true, " ") << "    "
                      << PrettyPrint::PrettyPrint(deserialized.first, true, " ")
                      << std::endl;
            assert(pose == deserialized.first);
        }
    }
}

// Tests header, transform, vector3, quaternion, and empty string
void TestTransformStamped(std::mt19937_64& generator)
{
    geometry_msgs::TransformStamped transform;
    transform.header = RandomHeader(generator);
    transform.transform = RandomTransform(generator);
    transform.child_frame_id = "";

    // Manual version
    {
        auto buffer = std::vector<uint8_t>();
        auto bytes_written = SerializeTransformStamped(transform, buffer);
        auto deserialized = DeserializeTransformStamped(buffer, 0);
        if (bytes_written != deserialized.second)
        {
            std::cerr << "Bytes written does not match bytes read "
                      << bytes_written << " " << deserialized.second;
            assert(bytes_written == deserialized.second);
        }
        if (!(transform == deserialized.first))
        {
            std::cerr << "Pre serialized data does not match deserialized data "
                      << PrettyPrint::PrettyPrint(transform, true, " ") << "    "
                      << PrettyPrint::PrettyPrint(deserialized.first, true, " ")
                      << std::endl;
            assert(transform ==deserialized.first);
        }
    }
    // Relying on ROS serialization
    {
        auto buffer = std::vector<uint8_t>();
        auto bytes_written = RosMessageSerializationWrapper(transform, buffer);
        auto deserialized = RosMessageDeserializationWrapper<geometry_msgs::TransformStamped>(buffer, 0);
        if (bytes_written != deserialized.second)
        {
            std::cerr << "Bytes written does not match bytes read "
                      << bytes_written << " " << deserialized.second;
            assert(bytes_written == deserialized.second);
        }
        if (!(transform == deserialized.first))
        {
            std::cerr << "Pre serialized data does not match deserialized data "
                      << PrettyPrint::PrettyPrint(transform, true, " ") << "    "
                      << PrettyPrint::PrettyPrint(deserialized.first, true, " ")
                      << std::endl;
            assert(transform == deserialized.first);
        }
    }
}

// Tests Marker, and all sub types
void TestVisualizationMarker(std::mt19937_64& generator)
{
    auto marker = RandomMarker(generator);
    auto buffer = std::vector<uint8_t>();
    auto bytes_written = RosMessageSerializationWrapper(marker, buffer);
    auto deserialized = RosMessageDeserializationWrapper<visualization_msgs::Marker>(buffer, 0);
    if (bytes_written != deserialized.second)
    {
        std::cerr << "Bytes written does not match bytes read "
                  << bytes_written << " " << deserialized.second;
        assert(bytes_written == deserialized.second);
    }
    if (!(marker == deserialized.first))
    {
        std::cerr << "Pre serialized data does not match deserialized data "
                  << PrettyPrint::PrettyPrint(marker, true, " ") << "    "
                  << PrettyPrint::PrettyPrint(deserialized.first, true, " ")
                  << std::endl;
        assert(marker == deserialized.first);
    }
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    const auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937_64 generator(seed);

    for (int i = 0; i < 100; ++i)
    {
        TestFixedSizePOD<bool>(false);
        TestFixedSizePOD<bool>(true);
        TestFixedSizeIntPOD<uint8_t>(generator);
        TestFixedSizeIntPOD<uint16_t>(generator);
        TestFixedSizeIntPOD<uint32_t>(generator);
        TestFixedSizeIntPOD<uint64_t>(generator);
        TestFixedSizeIntPOD<int8_t>(generator);
        TestFixedSizeIntPOD<int16_t>(generator);
        TestFixedSizeIntPOD<int32_t>(generator);
        TestFixedSizeIntPOD<int64_t>(generator);
        TestFixedSizeFloatPOD<float>(generator);
        TestFixedSizeFloatPOD<double>(generator);

        TestIntVector<uint8_t>(generator);
        TestIntVector<uint16_t>(generator);
        TestIntVector<uint32_t>(generator);
        TestIntVector<uint64_t>(generator);
        TestIntVector<int8_t>(generator);
        TestIntVector<int16_t>(generator);
        TestIntVector<int32_t>(generator);
        TestIntVector<int64_t>(generator);
        TestFloatVector<float>(generator);
        TestFloatVector<double>(generator);

        TestIntFloatMap<int, float>(generator);
        TestFloatIntMap<double, uint64_t>(generator);

        TestPoseStamped(generator);
        TestTransformStamped(generator);

        TestVisualizationMarker(generator);
    }

    std::cout << "All tests passed" << std::endl;
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
}

