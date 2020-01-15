#include "imutils.h"
#include <vcp_utils/vcp_error.h>
#include <vcp_utils/file_utils.h>
#include <fstream>
#include <cstdio>
#include <zlib.h>
#include <cstring>

// Allow a buffer of 16 KB to zip/unzip a cv::Mat
#define ZIP_MAT_BUFFER_SIZE 16384

namespace vcp
{
namespace imutils
{
namespace
{
char IsBigEndian()
{
  // https://stackoverflow.com/a/1001373
  union {
    uint32_t i;
    char c[4];
  } bint = {0x01020304};
  if (bint.c[0] == 1)
    return 1;
  return 0;
}

char MatDepthToIdentifier(int depth)
{
  switch(depth)
  {
  case CV_8U: return 'u';
  case CV_8S: return 'c';
  case CV_16U: return 'v';
  case CV_16S: return 's';
  case CV_32S: return 'i';
  case CV_32F: return 'f';
  case CV_64F: return 'd';
  default:
    VCP_ERROR("MatDepthToIdentifier(): Depth '" << depth << "' is not yet supported!");
  }
}

size_t SizeOfMatDepth(int depth)
{
  switch(depth)
  {
  case CV_8U: return sizeof(unsigned char);
  case CV_8S: return sizeof(char);
  case CV_16U: return sizeof(unsigned short);
  case CV_16S: return sizeof(short);
  case CV_32S: return sizeof(int);
  case CV_32F: return sizeof(float);
  case CV_64F: return sizeof(double);
  default:
    VCP_ERROR("SizeOfMatDepth(): Depth '" << depth << "' is not yet supported!");
  }
}

int MatDepthFromIdentifier(char id)
{
  switch(id)
  {
  case 'u': return CV_8U;
  case 'c': return CV_8S;
  case 'v': return CV_16U;
  case 's': return CV_16S;
  case 'i': return CV_32S;
  case 'f': return CV_32F;
  case 'd': return CV_64F;
  default:
    VCP_ERROR("MatDepthFromIdentifier(): Invalid depth identifier '" << id << "'");
  }
}
} // namespace

cv::Mat ColumnStack(const cv::Mat &A, const cv::Mat &B)
{
  VCP_CHECK(A.type() == B.type());
  VCP_CHECK(A.rows == B.rows);
  const int width = A.cols + B.cols;
  cv::Mat C(A.rows, width, A.type());
  cv::Mat roi = C.colRange(0, A.cols);
  A.copyTo(roi);
  roi = C.colRange(A.cols, width);
  B.copyTo(roi);
  return C;
}


cv::Mat RowStack(const cv::Mat &A, const cv::Mat &B)
{
  VCP_CHECK(A.type() == B.type());
  VCP_CHECK(A.cols == B.cols);
  const int height = A.rows + B.rows;
  cv::Mat C(height, A.cols, A.type());
  cv::Mat roi = C.rowRange(0, A.rows);
  A.copyTo(roi);
  roi = C.rowRange(A.rows, height);
  B.copyTo(roi);
  return C;
}


void ResizeMatrix(cv::Mat &mat, int rows, int cols, double value)
{
  cv::Mat resized(rows, cols, mat.type(), cv::Scalar::all(value));

  const int w = std::min(cols, mat.cols);
  const int h = std::min(rows, mat.rows);
  if (w > 0 && h > 0)
  {
    const cv::Rect roi(0, 0, w, h);
    mat(roi).copyTo(resized(roi));
  }

  mat = resized;
}


// Print variable name https://stackoverflow.com/a/7766138/400948
#define MAKE_DEPTH_TO_STRING_CASE(T, ss) case T: ss << #T; break;
std::string CVMatDepthToString(int depth, int channels)
{
  std::stringstream str;
  switch (CV_MAT_DEPTH(depth))
  {
  MAKE_DEPTH_TO_STRING_CASE(CV_8U, str);
  MAKE_DEPTH_TO_STRING_CASE(CV_8S, str);
  MAKE_DEPTH_TO_STRING_CASE(CV_16U, str);
  MAKE_DEPTH_TO_STRING_CASE(CV_16S, str);
  MAKE_DEPTH_TO_STRING_CASE(CV_32S, str);
  MAKE_DEPTH_TO_STRING_CASE(CV_32F, str);
  MAKE_DEPTH_TO_STRING_CASE(CV_64F, str);
  MAKE_DEPTH_TO_STRING_CASE(CV_USRTYPE1, str);

  default:
    str << "Unknown depth[" << CV_MAT_DEPTH(depth) << "]";
    break;
  }
  if (channels > 0)
    str << "C" << channels;
  return str.str();
}

bool DumpZippedMat(const cv::Mat &mat, const std::string &filename)
{
#ifdef VCP_WITH_ZLIB
  // Based on the official C example on zipped pipes: https://zlib.net/zpipe.c
  const char big_endian = IsBigEndian();
  const char id = MatDepthToIdentifier(mat.depth());
  const int height = mat.rows;
  const int width = mat.cols;
  const int layers = mat.channels();
  const size_t mat_data_size = height * width * layers * SizeOfMatDepth(mat.depth());
  const size_t dump_size = 2 * sizeof(char) + 3 * sizeof(int) + mat_data_size;

  // Copy header + cv::Mat data to temporary buffer.
  unsigned char *dumped_data = new unsigned char[dump_size];
  if (!dumped_data)
  {
    VCP_LOG_FAILURE("DumpZippedMat(): Cannot allocate buffer to zip the cv::Mat");
    return false;
  }
  std::memcpy(dumped_data, &id, sizeof(char));
  size_t offset = sizeof(char);
  std::memcpy(dumped_data + offset, &big_endian, sizeof(char));
  offset += sizeof(char);
  std::memcpy(dumped_data + offset, &height, sizeof(int));
  offset += sizeof(int);
  std::memcpy(dumped_data + offset, &width, sizeof(int));
  offset += sizeof(int);
  std::memcpy(dumped_data + offset, &layers, sizeof(int));
  offset += sizeof(int);
  std::memcpy(dumped_data + offset, mat.data, mat_data_size);

  // Initialize zip stream.
  const int zip_level = Z_DEFAULT_COMPRESSION;
  int ret;
  size_t have;
  z_stream strm;
  unsigned char out_buffer[ZIP_MAT_BUFFER_SIZE];

  // Allocate deflate state
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  ret = deflateInit(&strm, zip_level);
  if (ret != Z_OK)
  {
    VCP_LOG_FAILURE("DumpZippedMat(): Cannot initialize Zip stream!");
    delete [] dumped_data;
    return false;
  }

  std::ofstream file(filename + ".cvz", std::ios::out | std::ios::binary);
  if (!file.is_open())
  {
    VCP_LOG_FAILURE("DumpZippedMat(): Cannot open output file: " << filename << ".cvz");
    delete [] dumped_data;
    return false;
  }

  const int flush = Z_FINISH; // Once the temporary dumped_data has been compressed, we're done.
  strm.avail_in = dump_size;
  strm.next_in = dumped_data;

  // Run deflate() on input buffer (dumped_data) until output buffer not full, finish compression if all of source has been read in.
  do
  {
    strm.avail_out = ZIP_MAT_BUFFER_SIZE;
    strm.next_out = out_buffer;
    ret = deflate(&strm, flush);
    if (ret == Z_STREAM_ERROR)
    {
      VCP_LOG_FAILURE("DumpZippedMat(): Zip stream error while zipping cv::Mat buffer!");
      delete[] dumped_data;
      file.close();
      return false;
    }
    have = ZIP_MAT_BUFFER_SIZE - strm.avail_out;
    file.write(reinterpret_cast<const char *>(out_buffer), have);
  } while (strm.avail_out == 0);

  // Check that all input bytes have been processed.
  if (strm.avail_in != 0)
  {
    VCP_LOG_FAILURE("DumpZippedMat(): Zip stream didn't compress all of the cv::Mat buffer!");
  }

  // Clean up
  deflateEnd(&strm);
  delete[] dumped_data;
  file.close();
  return true;
#else // VCP_WITH_ZLIB
  VCP_LOG_FAILURE("DumpZippedMat(): VCP needs to be build with ZLIB to support storing compressed raw images");
  return false;
#endif // VCP_WITH_ZLIB
}

bool DumpMat(const cv::Mat &mat, const std::string &filename, bool zip)
{
  if (zip)
  {
    return DumpZippedMat(mat, filename);
  }
  else
  {
    const char big_endian = IsBigEndian();
    const char id = MatDepthToIdentifier(mat.depth());
    const int height = mat.rows;
    const int width = mat.cols;
    const int layers = mat.channels();

    std::ofstream file(filename + ".cvm", std::ios::out | std::ios::binary);
    if (!file.is_open())
    {
      VCP_LOG_FAILURE("DumpMat(): Cannot open output file: " << filename << ".cvm");
      return false;
    }
    file.write(&id, 1);
    file.write(&big_endian, 1);
    file.write(reinterpret_cast<const char *>(&height), sizeof(height));
    file.write(reinterpret_cast<const char *>(&width), sizeof(width));
    file.write(reinterpret_cast<const char *>(&layers), sizeof(layers));
    file.write(reinterpret_cast<const char *>(mat.data), width * height * layers * SizeOfMatDepth(mat.depth()));
    file.close();
    VCP_LOG_DEBUG("DumpMat(): Stored image to disk: " << id << ", " << int(big_endian) << ", " << width << "x" << height << ", " << layers << " channels");
  }
  return true;
}


class ZipMatReader
{
public:
  ZipMatReader()
  {
    header_size_ = 2*sizeof(char) + 3*sizeof(int);
    header_ = new unsigned char[header_size_];
    if (!header_)
    {
      VCP_EXIT("ZipMatReader() Cannot allocate memory to hold a zipped cv::Mat's header!", 1);
    }
    mat_size_ = 0;
    bytes_received_ = 0;
    header_offset_ = 0;
    mat_offset_ = 0;
  }

  virtual ~ZipMatReader()
  {
    if (header_)
      delete[] header_;
  }

  size_t append(const unsigned char *buffer, size_t num_bytes)
  {
    const size_t append_header = (bytes_received_ >= header_size_)
        ? 0 : std::min((header_size_ - bytes_received_), num_bytes);

    if (append_header > 0)
    {
      std::memcpy(header_ + header_offset_, buffer, append_header);
      header_offset_ += append_header;
      bytes_received_ += append_header;
      CheckHeader();
    }

    const size_t append_mat = std::min(mat_size_ - (bytes_received_ - header_size_), num_bytes - append_header);

    if (append_mat > 0)
    {
      std::memcpy(mat_.data + mat_offset_, buffer + append_header, append_mat);
      mat_offset_ += append_mat;
      bytes_received_ += append_mat;
    }

    const size_t appended = append_header + append_mat;
    if (appended != num_bytes)
    {
      VCP_ERROR("Appending inflated buffer would lead to buffer overflow! Expected to receive a "
         << mat_.cols << "x" << mat_.rows << " " << CVMatDepthToString(mat_.depth(), mat_.channels()) << " cv::Mat, and "
         << header_size_ << " bytes for the header. There are " << (num_bytes - appended) << " bytes too many!");
    }
    return append_header + append_mat;
  }

  cv::Mat GetMat() const { return mat_; }

private:
  size_t header_size_;
  size_t mat_size_;
  unsigned char *header_;
  size_t bytes_received_;
  size_t header_offset_;  // Offset into header buffer
  size_t mat_offset_;     // Offset into mat buffer
  cv::Mat mat_;

  void CheckHeader()
  {
    if (bytes_received_ != header_size_)
      return;

    const char *id = reinterpret_cast<const char *>(header_);
    size_t offset = sizeof(char);
    const char *file_big_endian = reinterpret_cast<const char*>(header_ + offset);
    offset += sizeof(char);
    const int *height = reinterpret_cast<const int*>(header_ + offset);
    offset += sizeof(int);
    const int *width = reinterpret_cast<const int*>(header_ + offset);
    offset += sizeof(int);
    const int *layers = reinterpret_cast<const int*>(header_ + offset);
    VCP_LOG_DEBUG("CheckHeader(): " << *id << ", " << (int)*file_big_endian << ", " << *width << "x" << *height << " with " << *layers << " channels");

    // Check endianness.
    const char platform_big_endian = IsBigEndian();
    if (platform_big_endian != *file_big_endian)
    {
      VCP_ERROR("CheckHeader(): Loading a binary file from a platform with different endianness is not yet supported!");
    }

    // Allocate mat buffer
    const int mat_depth = MatDepthFromIdentifier(*id);
    mat_ = cv::Mat(*height, *width, CV_MAKETYPE(mat_depth, *layers));
    mat_size_ = (*height) * (*width) * (*layers) * SizeOfMatDepth(mat_depth);
  }
};

cv::Mat LoadZippedMat(const std::string &filename)
{
#ifdef VCP_WITH_ZLIB
  int ret;
  unsigned have;
  z_stream strm;
  unsigned char in[ZIP_MAT_BUFFER_SIZE];
  unsigned char out[ZIP_MAT_BUFFER_SIZE];

  // C file I/O leads to less complex code when using zlib's API.
  FILE *source = fopen(filename.c_str(), "rb");
  if (!source)
  {
    VCP_LOG_FAILURE("LoadZippedMat(): Cannot open '" << filename << "'");
    return cv::Mat();
  }

  // Allocate inflate state.
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  strm.avail_in = 0;
  strm.next_in = Z_NULL;
  ret = inflateInit(&strm);
  if (ret != Z_OK)
  {
    VCP_LOG_FAILURE("LoadZippedMat(): Cannot initialize zip stream!");
    return cv::Mat();
  }

  ZipMatReader mat_reader;

  // Decompress until deflate stream ends or end of file.
  do
  {
    strm.avail_in = fread(in, 1, ZIP_MAT_BUFFER_SIZE, source);
    if (ferror(source))
    {
      VCP_LOG_FAILURE("LoadZippedMat(): Error while reading zipped cv::Mat!");
      inflateEnd(&strm);
      fclose(source);
      return cv::Mat();
    }

    if (strm.avail_in == 0)
        break;
    strm.next_in = in;

    // Inflate the input until output buffer not full.
    do
    {
      strm.avail_out = ZIP_MAT_BUFFER_SIZE;
      strm.next_out = out;
      ret = inflate(&strm, Z_NO_FLUSH);

      switch (ret)
      {
      case Z_NEED_DICT:
          ret = Z_DATA_ERROR;
          // fall through
      case Z_DATA_ERROR:
      case Z_MEM_ERROR:
      case Z_STREAM_ERROR:
          VCP_LOG_FAILURE("LoadZippedMat(): Error while inflating zipped buffer!");
          inflateEnd(&strm);
          fclose(source);
          return cv::Mat();
      }
      have = ZIP_MAT_BUFFER_SIZE - strm.avail_out;
      if (mat_reader.append(out, have) != have)
      {
        VCP_LOG_FAILURE("LoadZippedMat(): Error appending inflated zip buffer to cv::Mat");
        inflateEnd(&strm);
        fclose(source);
        return cv::Mat();
      }
    } while (strm.avail_out == 0);
    // We're done when inflate() says it's done.
  } while (ret != Z_STREAM_END);

  // Clean up.
  inflateEnd(&strm);
  fclose(source);
  if (ret == Z_STREAM_END)
    return mat_reader.GetMat();
  return cv::Mat();
#else // VCP_WITH_ZLIB
  VCP_LOG_FAILURE("LoadZippedMat(): VCP needs to be build with ZLIB to support loading compressed raw images");
  return cv::Mat();
#endif // VCP_WITH_ZLIB
}

cv::Mat LoadMat(const std::string &filename)
{
  const std::string ext = vcp::utils::file::GetExtension(filename);
  const bool is_zipped = ext.compare(".cvz") == 0;
  if (is_zipped)
  {
    return LoadZippedMat(filename);
  }
  const char platform_big_endian = IsBigEndian();
  char id, file_big_endian;
  int height, width, layers;

  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open())
  {
    VCP_LOG_FAILURE("LoadMat(): Cannot open input file: " << filename);
    return cv::Mat();
  }
  file.read(&id, 1);
  const int mat_depth = MatDepthFromIdentifier(id);
  file.read(&file_big_endian, 1);
  if (file_big_endian != platform_big_endian)
  {
    VCP_ERROR("LoadMat(): Loading a binary file from a platform with different endianness is not yet supported!");
  }
  file.read(reinterpret_cast<char *>(&height), sizeof(height));
  file.read(reinterpret_cast<char *>(&width), sizeof(width));
  file.read(reinterpret_cast<char *>(&layers), sizeof(layers));
  cv::Mat mat = cv::Mat(height, width, CV_MAKETYPE(mat_depth, layers));
  VCP_LOG_DEBUG("LoadMat(): Loading cv::Mat from disk: " << id << ", (" << int(file_big_endian) << " vs " << int(platform_big_endian) << ") " << width << "x" << height << ", " << layers << " channels");
  file.read(reinterpret_cast<char *>(mat.data), width * height * layers * SizeOfMatDepth(mat.depth()));
  file.close();
  return mat;
}

} // namespace imutils
} // namespace vcp
