// PointCloudExporter.hpp
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <tuple>
#include <algorithm>
// 引入 base_type.hpp 中的定义
#include "base_type.hpp"

// 辅助把宏值转成字符串
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// 打印 GCC 完整版本信息
#ifdef __VERSION__
#  pragma message("Compiler full version (__VERSION__): " __VERSION__)
#endif

// 打印各个版本宏
#pragma message("GCC major version: "   STR(__GNUC__))
#pragma message("GCC minor version: "   STR(__GNUC_MINOR__))
#pragma message("GCC patchlevel: "      STR(__GNUC_PATCHLEVEL__))

// 打印 C++ 标准模式
#pragma message("__cplusplus macro: "   STR(__cplusplus))

// 打印 bit_cast 特性宏（如果存在）
#ifdef __cpp_lib_bit_cast
#  pragma message("__cpp_lib_bit_cast: " STR(__cpp_lib_bit_cast))
#else
#  pragma message("__cpp_lib_bit_cast is not defined")
#endif
// 检查是否用的是 GCC，并且主版本 >= 10
#if !defined(__GNUC__) || (__GNUC__ < 10)
  #error "This library requires GCC version 10 or newer. Please upgrade your compiler."
#endif

// 检查是否启用了 C++20（包括 GNU 扩展模式 gnu++20）
// __cplusplus 在 -std=gnu++20 下会>=202002L
#if (__cplusplus < 202002L)
  #error "This library requires C++20. Please compile with -std=gnu++20 or higher."
#endif

// 可选：进一步检查 std::bit_cast 支持（C++20 标准库的一部分）
#if !defined(__cpp_lib_bit_cast) || (__cpp_lib_bit_cast < 201806L)
  #error "std::bit_cast not available: ensure you're using a C++20 standard library implementation."
#endif

// using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
// using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;

class PointCloudExporter {
public:
    // 使用 base_type 定义的类型
    using PointType = BasePointf;
    static constexpr size_t PACKET_MTU = 1400; // 可调整
    using PacketType = PointPacket<PointType, PACKET_MTU>;
    using PointPollType = PointPoll<PointType>;

    /**
     * @brief 构造函数
     * @param initial_pool_size 初始点池大小
     */
    explicit PointCloudExporter(size_t initial_pool_size = 1024);

    /**
     * @brief 插入 ROS 点云数据到内部缓冲区（转换为 BasePointf）
     * @param cloud 输入点云（PointXYZI 类型）
     */
    void addPoints(const PointCloudXYZI::ConstPtr& cloud);

  bool saveToBinaryFile(const std::string &filename, size_t num_threads);

    /**
     * @brief 导出点云为 PCD 文件
     * @param file_path 输出文件路径
     */
    // void saveToPCDFile(const std::string& file_path) const;

    /**
     * @brief 获取内部点池引用（可用于调试或其他处理）
     * @return const reference to PointPoll<BasePointf>
     */
    // const PointPollType& getPointPool() const;

  

private:
    /**
     * @brief 将点云序列化为多个二进制数据包（适合网络传输）
     * @param num_threads 多线程编码数量
     * @return shared_ptr<vector<span<byte>>> 数据包集合
     */
    // std::shared_ptr<std::vector<std::span<std::byte>>> serializeToPackets(size_t num_threads = 1);

        /**
     * @brief 获取当前缓存的所有点（BasePointf 格式）
     * @return vector of BasePointf
     */
    // std::vector<PointType> getRawPoints() const;

    std::tuple<int, int, int> intensityToHeatmap(float intensity);
    PointPollType point_poll_;
};

// Inline 实现放在 hpp 中以支持模板
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

inline PointCloudExporter::PointCloudExporter(size_t initial_pool_size): point_poll_{initial_pool_size}
{
    
}

inline void PointCloudExporter::addPoints(const PointCloudXYZI::ConstPtr& cloud)
{
    for (const auto& pt : cloud->points) {
        PointType base_pt;
        base_pt.xyz() << pt.x, pt.y, pt.z;
        float intensity = pt.intensity / 255.0f;
        auto [r, g, b] = intensityToHeatmap(intensity);
        base_pt.rgb() << r, g, b;
        base_pt.normal() << 0.0f, 0.0f, 0.0f;

        point_poll_.AddPoint(base_pt);
    }
}

// inline std::vector<PointCloudExporter::PointType> PointCloudExporter::getRawPoints() const
// {
//     return point_poll_._points;
// }

// inline std::shared_ptr<std::vector<std::span<std::byte>>> PointCloudExporter::serializeToPackets(size_t num_threads)
// {
//     // return point_poll_.EncodePackets<PacketType>(0, point_poll_.size(), num_threads);

// }

// inline void PointCloudExporter::saveToPCDFile(const std::string& file_path) const
// {
//     pcl::PointCloud<pcl::PointXYZRGB> cloud;
//     cloud.reserve(point_poll_.size());

//     for (const auto& pt : point_poll_._points) {
//         pcl::PointXYZRGB pcl_pt;
//         pcl_pt.x = pt._pos[0];
//         pcl_pt.y = pt._pos[1];
//         pcl_pt.z = pt._pos[2];
//         pcl_pt.r = static_cast<uint8_t>(pt._color[0] * 255);
//         pcl_pt.g = static_cast<uint8_t>(pt._color[1] * 255);
//         pcl_pt.b = static_cast<uint8_t>(pt._color[2] * 255);
//         cloud.push_back(pcl_pt);
//     }

//     if (pcl::io::savePCDFile(file_path, cloud) == 0) {
//         std::cout << "Saved PCD file: " << file_path << std::endl;
//     } else {
//         std::cerr << "Failed to save PCD file: " << file_path << std::endl;
//     }
// }

// inline const PointCloudExporter::PointPollType& PointCloudExporter::getPointPool() const
// {
//     return point_poll_;
// }

inline std::tuple<int, int, int> PointCloudExporter::intensityToHeatmap(float intensity) {
        intensity = std::max(0.0f, std::min(255.0f, intensity));

        int r, g, b;

        if (intensity < 128.0f) {
            r = 0;
            g = static_cast<int>(255.0f * intensity / 128.0f);
            b = 255;
        } else {
            float scaled = (intensity - 128.0f) / 127.0f;
            r = static_cast<int>(255.0f * scaled);
            g = 255;
            b = static_cast<int>(255.0f - 255.0f * scaled);
        }

        return std::make_tuple(r, g, b);
    }

inline bool PointCloudExporter::saveToBinaryFile(const std::string &filename, size_t num_threads)
{
    // Step 1: 序列化为数据包
    // auto packets = this->serializeToPackets(num_threads);
    std::vector<std::byte> buffer;
    auto out = zpp::bits::out(buffer,zpp::bits::endian::big{});
    out(point_poll_).or_throw();

    // Step 2: 打开二进制文件准备写入
    std::ofstream file(filename, std::ios::out | std::ios::binary);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    // // Step 3: 写入每个 packet 的内容到文件中
    // // for (const auto &packet_span : *packets)
    // // {
    //     file.write(reinterpret_cast<const char *>(packet_span.data()), packet_span.size());
    //     if (!file.good())
    //     {
    //         std::cerr << "Error writing to file: " << filename << std::endl;
    //         file.close();
    //         return false;
    //     }
    // // }

     file.write(
      reinterpret_cast<const char *>(buffer.data()),
      static_cast<std::streamsize>(buffer.size())
    );

    // 4. 检查写入状态
    if (!file.good()) {
        std::cerr << "Error writing to file: " << filename << std::endl;
        file.close();
        return false;
    }

    file.close();
    return true;
}