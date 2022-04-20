/**
 * Copyright (c) 2019, Horizon Robotics, Inc.
 * All rights reserved.
 * @Author: yong.wu
 * @Mail: yong.wu@horizon.ai
 * @Date: 2020-06-29
 * @Version: v0.0.1
 * @Brief: implemenation of iot media codec .
 */

#include <string.h>
#include <unistd.h>
#include <thread>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "utils/time_helper.h"
#include "media_codec/media_codec_manager.h"
#include "hb_vp_api.h"

#define JPEG_QUALITY_FACTOR_DEFAULT_VALUE   50
#define MAX_POOL_CNT 32

using hobot::Timer;
namespace horizon {
namespace vision {

int MediaCodecManager::SetDefaultChnAttr(VENC_CHN_ATTR_S* pVencChnAttr,
        PAYLOAD_TYPE_E type, int width,
        int height, int frame_buf_depth,
        PIXEL_FORMAT_E pix_fmt,
        int frame_rate) {
    memset(pVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    pVencChnAttr->stVencAttr.enType = type;

    pVencChnAttr->stVencAttr.u32PicWidth = width;
    pVencChnAttr->stVencAttr.u32PicHeight = height;

    pVencChnAttr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
    pVencChnAttr->stVencAttr.enRotation = CODEC_ROTATION_0;
    pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;

    if (type == PT_JPEG || type == PT_MJPEG) {
        pVencChnAttr->stVencAttr.enPixelFormat = pix_fmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = frame_buf_depth;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = frame_buf_depth;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
        pVencChnAttr->stVencAttr.stAttrJpeg.quality_factor =
            JPEG_QUALITY_FACTOR_DEFAULT_VALUE;
        pVencChnAttr->stVencAttr.stAttrJpeg.restart_interval = 0;
    } else {
        pVencChnAttr->stVencAttr.enPixelFormat = pix_fmt;
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = frame_buf_depth;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = frame_buf_depth;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
    }

    if (type == PT_H265) {
#ifdef H265_LOSSLESS
      m_VencChnAttr.stVencAttr.stAttrH265.lossless_mode = 1;
      m_VencChnAttr.stVencAttr.stAttrH265.transform_skip_enabled_flag = 1;
#endif
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        pVencChnAttr->stRcAttr.stH265Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraPeriod = 20;
        pVencChnAttr->stRcAttr.stH265Vbr.u32FrameRate = frame_rate;
    }
    if (type == PT_H264) {
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
        pVencChnAttr->stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH264Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH264Vbr.u32IntraPeriod = 10;
        pVencChnAttr->stRcAttr.stH264Vbr.u32FrameRate = frame_rate;
    }

    pVencChnAttr->stGopAttr.u32GopPresetIdx = 2;
    pVencChnAttr->stGopAttr.s32DecodingRefreshType = 2;

    return 0;
}

int MediaCodecManager::SetDefaultRcParams(int chn, PAYLOAD_TYPE_E type,
        int width, int height, bool is_cbr,
        int bitrate, VENC_CHN_ATTR_S* attr,
        int framerate) {
    int ret;

    if (type == PT_H264) {
        if (is_cbr) {
            attr->stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
            VENC_RC_ATTR_S *pstRcParam = &(attr->stRcAttr);
            ret = HB_VENC_GetRcParam(chn, pstRcParam);
            if (ret) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "H264 HB_VENC_GetRcParam "
                     << chn << " failed: " << ret);
                return ret;
            }
            pstRcParam->stH264Cbr.u32BitRate = bitrate;
            pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
            pstRcParam->stH264Cbr.u32FrameRate = framerate;
            pstRcParam->stH264Cbr.u32IntraPeriod = 10;
        } else {
            attr->stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
            VENC_RC_ATTR_S *pstRcParam = &(attr->stRcAttr);
            ret = HB_VENC_GetRcParam(chn, pstRcParam);
            if (ret) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "H264 HB_VENC_GetRcParam "
                     << chn << " failed: " << ret);
                return ret;
            }
            pstRcParam->stH264Vbr.u32FrameRate = framerate;
            pstRcParam->stH264Vbr.u32IntraPeriod = 10;
        }
    } else if (type == PT_H265) {
        attr->stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        VENC_RC_ATTR_S *pstRcParam = &(attr->stRcAttr);
        ret = HB_VENC_GetRcParam(chn, pstRcParam);
        if (ret) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "H265 HB_VENC_GetRcParam " << chn << " failed: " << ret);
            return ret;
        }
        pstRcParam->stH265Cbr.u32BitRate = bitrate;
        pstRcParam->stH265Cbr.u32FrameRate = framerate;
        pstRcParam->stH265Cbr.u32IntraPeriod = 30;
        pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
    } else if (type == PT_MJPEG) {
        attr->stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
        VENC_RC_ATTR_S *pstRcParam = &(attr->stRcAttr);
        ret = HB_VENC_GetRcParam(chn, pstRcParam);
        if (ret) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "MJPG HB_VENC_GetRcParam " << chn << " failed: " << ret);
            return ret;
        }
        pstRcParam->stMjpegFixQp.u32FrameRate = framerate;
        pstRcParam->stMjpegFixQp.u32QualityFactort = bitrate;
    }

    return 0;
}

int MediaCodecManager::SetUserCbrParams(int chn, int bit_rate, int fps,
        int intra, int buf_size) {
    int ret;
    PAYLOAD_TYPE_E type = chn_type_[chn];
    VENC_CHN_ATTR_S *attr = &venc_chn_attr_[chn];
    VENC_RC_ATTR_S *pstRcParam = nullptr;

    if (type == PT_H264)  {
        attr->stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        pstRcParam = &(attr->stRcAttr);
        ret = HB_VENC_GetRcParam(chn, pstRcParam);
        if (ret) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "H264 HB_VENC_GetRcParam " << chn << " failed: " << ret);
            return ret;
        }
        pstRcParam->stH264Cbr.u32BitRate = bit_rate;
        pstRcParam->stH264Cbr.u32FrameRate = fps;
        pstRcParam->stH264Cbr.u32IntraPeriod = intra;
        pstRcParam->stH264Cbr.u32VbvBufferSize = buf_size;
    } else if (type == PT_H265) {
        attr->stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        pstRcParam = &(attr->stRcAttr);
        ret = HB_VENC_GetRcParam(chn, pstRcParam);
        if (ret) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "H265 HB_VENC_GetRcParam " << chn << " failed: " << ret);
            return ret;
        }
        pstRcParam->stH265Cbr.u32BitRate = bit_rate;
        pstRcParam->stH265Cbr.u32FrameRate = fps;
        pstRcParam->stH265Cbr.u32IntraPeriod = intra;
        pstRcParam->stH265Cbr.u32VbvBufferSize = buf_size;
    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "not support chn type: " << type);
        return -1;
    }

    ret = HB_VENC_SetRcParam(chn, pstRcParam);
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "HB_VENC_SetRcParam " << chn << " failed: " << ret);
        return ret;
    }


    return 0;
}

int MediaCodecManager::SetUserQfactorParams(int chn, int value) {
    int ret;
    VENC_JPEG_PARAM_S *jpg_params = &venc_chn_jpg_params_[chn];

    ret = HB_VENC_GetJpegParam(chn, jpg_params);
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "set qfactor, HB_VENC_GetJpegParam " << chn
             << " failed: " << ret);
        return ret;
    }
    jpg_params->u32Qfactor = value;
    ret = HB_VENC_SetJpegParam(chn, jpg_params);
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "set qfactor, HB_VENC_SetJpegParam " << chn
             << " failed: " << ret);
        return ret;
    }

    return 0;
}

int MediaCodecManager::EncodeChnInit(int chn, PAYLOAD_TYPE_E type,
        int width, int height, int frame_buf_depth, PIXEL_FORMAT_E pix_fmt,
        bool is_cbr, int bitrate, int framerate) {
    std::lock_guard<std::mutex> lg(mutex_);
    int ret;
    VENC_CHN_ATTR_S VencChnAttr;

    /* 1. Set Venc chn Attr */
    SetDefaultChnAttr(&VencChnAttr, type, width, height,
            frame_buf_depth, pix_fmt, framerate);

    /* 2. Create Venc chn */
    ret = HB_VENC_CreateChn(chn, &VencChnAttr);
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "HB_VENC_CreateChn, chn: " << chn << " failed: " << ret);
        return ret;
    }

    /* 3. Set default rc params */
    ret = SetDefaultRcParams(chn, type, width, height,
      is_cbr, bitrate, &VencChnAttr, framerate);
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "SetDefaultCbrParams, chn: " << chn << " failed: " << ret);
        return ret;
    }

    /* 4. Set encoder chn attr */
    ret = HB_VENC_SetChnAttr(chn, &VencChnAttr);
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "HB_VENC_SetChnAttr, chn " << chn << " failed: " << ret);
        return ret;
    }
    venc_chn_attr_[chn] = VencChnAttr;
    chn_type_[chn] = type;

    return 0;
}

int MediaCodecManager::EncodeChnDeInit(int chn) {
    std::lock_guard<std::mutex> lg(mutex_);
    int ret;

    ret = HB_VENC_DestroyChn(chn);
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "HB_VENC_DestroyChn, chn " << chn << " failed: " << ret);
        return ret;
    }

    return 0;
}

int MediaCodecManager::EncodeChnStart(int chn) {
    int ret;

    VENC_RECV_PIC_PARAM_S pstRecvParam;
    pstRecvParam.s32RecvPicNum = 0;

    ret = HB_VENC_StartRecvFrame(chn, &pstRecvParam);
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "HB_VENC_StartRecvFrame, chn " << chn << " failed: " << ret);
        return ret;
    }

    return 0;
}

int MediaCodecManager::EncodeChnStop(int chn) {
    int ret;

    ret = HB_VENC_StopRecvFrame(chn);
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "HB_VENC_StopRecvFrame, chn " << chn << " failed: " << ret);
        return ret;
    }

    return 0;
}

int MediaCodecManager::EncodeYuvToJpg(int chn, iot_venc_src_buf_t *yuv_buf,
                                      std::vector<uint8_t> &jpg_result) {
  iot_venc_stream_buf_t *stream_buf = nullptr;
  int ret = EncodeYuvToJpg(chn, yuv_buf, &stream_buf);
  if (ret != 0) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "encode yuv to jpg failed");
    return -1;
  }
  jpg_result.clear();
  auto data_ptr = stream_buf->stream_info.pstPack.vir_ptr;
  auto data_size = stream_buf->stream_info.pstPack.size;
  jpg_result.assign(data_ptr, data_ptr + data_size);
  if (stream_buf != nullptr) {
    ret = FreeStream(chn, stream_buf);
    if (ret != 0) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "FreeStream failed");
    }
    stream_buf = nullptr;
  }
  return 0;
}

int MediaCodecManager::EncodeYuvToJpg(int chn, iot_venc_src_buf_t *src_buf,
        iot_venc_stream_buf_t **stream_buf) {
    // 添加mutex的目的是确保HB_VENC_SendFrame 与 HB_VENC_GetStream能够对应,
    // 避免多线程调用EncodeYuvToJpg出现错乱
    std::lock_guard<std::mutex> lg(mutex_);
    int ret;
    VIDEO_FRAME_S pstFrame = { 0 };
    iot_venc_stream_buf_t *venc_buf;


    /* cache flush */
    if (use_vb_[chn] && vb_cache_en_[chn]) {
      uint32_t stride, height;
      uint32_t size, size_y, size_uv, size_u, size_v;
      uint32_t phy_ptr0, phy_ptr1, phy_ptr2;
      char *vir_ptr0, *vir_ptr1, *vir_ptr2;

      if (vb_stream_buf_queue_[chn].size() == 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "vb stream buf queue size is zero, error!");
        return -1;
      }
      venc_buf = vb_stream_buf_queue_[chn].front();
      vb_stream_buf_queue_[chn].pop();
      if (venc_buf == nullptr) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "vb buf is nullptr");
        return -1;
      }
      size = src_buf->frame_info.size;
      height = src_buf->frame_info.height;
      stride = src_buf->frame_info.stride;
      size_y = stride * height;
      size_uv = size - size_y;
      size_u = size_uv / 2;
      size_v = size_uv / 2;
      phy_ptr0 = src_buf->frame_info.phy_ptr[0];
      vir_ptr0 = src_buf->frame_info.vir_ptr[0];
      phy_ptr1 = src_buf->frame_info.phy_ptr[1];
      vir_ptr1 = src_buf->frame_info.vir_ptr[1];
      phy_ptr2 = src_buf->frame_info.phy_ptr[2];
      vir_ptr2 = src_buf->frame_info.vir_ptr[2];
      PIXEL_FORMAT_E pix_fmt = venc_chn_attr_[chn].stVencAttr.enPixelFormat;
      if (pix_fmt == HB_PIXEL_FORMAT_NV12 ||
          pix_fmt == HB_PIXEL_FORMAT_NV21 ||
          pix_fmt == HB_PIXEL_FORMAT_NV16 ||
          pix_fmt == HB_PIXEL_FORMAT_NV61) {
        ret = HB_SYS_CacheFlush(phy_ptr0, vir_ptr0, size_y);
        if (ret) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "pix_fmt:" << pix_fmt << " cache flush y failed, ret:" << ret);
          return ret;
        }
        ret = HB_SYS_CacheFlush(phy_ptr1, vir_ptr1, size_uv);
        if (ret) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "pix_fmt:" << pix_fmt
               << " cache flush uv failed, ret:" << ret);
          return ret;
        }
      } else if (pix_fmt == HB_PIXEL_FORMAT_YUYV422 ||
          pix_fmt == HB_PIXEL_FORMAT_YVYU422 ||
          pix_fmt == HB_PIXEL_FORMAT_UYVY422 ||
          pix_fmt == HB_PIXEL_FORMAT_VYUY422 ||
          pix_fmt == HB_PIXEL_FORMAT_YUV400) {
        ret = HB_SYS_CacheFlush(phy_ptr0, vir_ptr0, size);
        if (ret) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "pix_fmt:" << pix_fmt
               << " cache flush yuv failed, ret:" << ret);
          return ret;
        }
      } else if (pix_fmt == HB_PIXEL_FORMAT_YUV422P ||
          pix_fmt == HB_PIXEL_FORMAT_YUV420P) {
        ret = HB_SYS_CacheFlush(phy_ptr0, vir_ptr0, size_y);
        if (ret) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "pix_fmt:" << pix_fmt << " cache flush y failed, ret:" << ret);
          return ret;
        }
        ret = HB_SYS_CacheFlush(phy_ptr1, vir_ptr1, size_u);
        if (ret) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "pix_fmt:" << pix_fmt << " cache flush u failed, ret:" << ret);
          return ret;
        }
        ret = HB_SYS_CacheFlush(phy_ptr2, vir_ptr2, size_v);
        if (ret) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "pix_fmt:" << pix_fmt << " cache flush v failed, ret:" << ret);
          return ret;
        }
      } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "no support JPEG encode format!!!");
        return -1;
      }
    } else {
      venc_buf = static_cast<iot_venc_stream_buf_t*>\
                 (malloc(sizeof(iot_venc_stream_buf_t)));
      if (nullptr == venc_buf) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "malloc failed");
      }
    }

    memset(venc_buf, 0x00, sizeof(iot_venc_stream_buf_t));
    pstFrame.stVFrame = src_buf->frame_info;
    while (true) {
        ret = HB_VENC_SendFrame(chn, &pstFrame, 3000);
        if (ret < 0) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "HB_VENC_SendFrame, chn " << chn << " failed: " << ret);
            // std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } else {
            break;
        }
    }

    ret = HB_VENC_GetStream(chn, &venc_buf->stream_info, 3000);
    if (ret != 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "HB_VENC_GetStream, chn " << chn << " failed: " << ret);
        if (use_vb_[chn]) {
          vb_stream_buf_queue_[chn].push(
              venc_buf);  // push again for cycle queue
        } else {
            free(venc_buf);
        }
        *stream_buf = nullptr;
        return -1;
    } else {
        venc_buf->chn = chn;
        venc_buf->src_width = src_buf->frame_info.width;
        venc_buf->src_height = src_buf->frame_info.height;
        venc_buf->src_pix_format = src_buf->frame_info.pix_format;
        venc_buf->src_pts = src_buf->frame_info.pts;
        *stream_buf = venc_buf;
        return 0;
    }
}

int MediaCodecManager::FreeStream(int chn, iot_venc_stream_buf_t *stream_buf) {
    std::lock_guard<std::mutex> lg(mutex_);
    int ret;

    // HOBOT_CHECK(stream_buf != nullptr) << "stream buf is null";

    ret = HB_VENC_ReleaseStream(chn, &stream_buf->stream_info);
    if (ret < 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "HB_VENC_ReleaseStream, chn " << chn << " failed: " << ret);
    }

    if (use_vb_[chn]) {
      vb_stream_buf_queue_[chn].push(stream_buf);  // push again for cycle queue
    } else {
      free(stream_buf);
    }

    return 0;
}

int MediaCodecManager::AllocVbBuf2Lane(int index, void *buf,
        int size_y, int size_uv, int vb_cache_en) {
    int ret;
    iot_venc_src_buf_t *buffer = reinterpret_cast<iot_venc_src_buf_t*>(buf);
    uint64_t phy_ptr0, phy_ptr1;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "enter " << __FUNCTION__);
    if (buffer == nullptr) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "buffer is nullptr error!");
        return -1;
    }

    if (vb_cache_en == 1) {
      ret = HB_SYS_AllocCached(&phy_ptr0,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[0]), size_y);
    } else {
      ret = HB_SYS_Alloc(&phy_ptr0,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[0]), size_y);
    }
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "index: " << index << "alloc vb buffer error, ret: " << ret);
        return ret;
    }
    buffer->frame_info.phy_ptr[0] = static_cast<uint32_t>(phy_ptr0);

    if (vb_cache_en == 1) {
      ret = HB_SYS_AllocCached(&phy_ptr1,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[1]), size_uv);
    } else {
      ret = HB_SYS_Alloc(&phy_ptr1,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[1]), size_uv);
    }
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "index: " << index << "alloc vb buffer error, ret: " << ret);
        return ret;
    }
    buffer->frame_info.phy_ptr[1] = static_cast<uint32_t>(phy_ptr1);

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "mmzAlloc index: " << index);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "vio_buf_addr: "   << buffer);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_y_paddr: "    << buffer->frame_info.phy_ptr[0]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_y_vaddr "
         << reinterpret_cast<void*>(buffer->frame_info.vir_ptr[0]));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_uv_paddr: "   << buffer->frame_info.phy_ptr[1]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_uv_vaddr: "
         << reinterpret_cast<void*>(buffer->frame_info.vir_ptr[1]));

    return 0;
}

int MediaCodecManager::AllocVbBuf3Lane(int index, void *buf,
        int size_y, int size_u, int size_v, int vb_cache_en) {
    int ret;
    iot_venc_src_buf_t *buffer = reinterpret_cast<iot_venc_src_buf_t*>(buf);
    uint64_t phy_ptr0, phy_ptr1, phy_ptr2;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "enter " << __FUNCTION__);
    if (buffer == nullptr) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "buffer is nullptr error!");
        return -1;
    }

    if (vb_cache_en == 1) {
      ret = HB_SYS_AllocCached(&phy_ptr0,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[0]), size_y);
    } else {
      ret = HB_SYS_Alloc(&phy_ptr0,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[0]), size_y);
    }
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "index: " << index << "alloc vb y buffer error, ret: " << ret);
        return ret;
    }
    buffer->frame_info.phy_ptr[0] = static_cast<uint32_t>(phy_ptr0);

    if (vb_cache_en == 1) {
      ret = HB_SYS_AllocCached(&phy_ptr1,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[1]), size_u);
    } else {
      ret = HB_SYS_Alloc(&phy_ptr1,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[1]), size_u);
    }
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "index: " << index << "alloc vb u buffer error, ret: " << ret);
        return ret;
    }
    buffer->frame_info.phy_ptr[1] = static_cast<uint32_t>(phy_ptr1);

    if (vb_cache_en == 1) {
      ret = HB_SYS_AllocCached(&phy_ptr2,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[2]), size_v);
    } else {
      ret = HB_SYS_Alloc(&phy_ptr2,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[2]), size_v);
    }
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "index: " << index << "alloc vb v buffer error, ret: " << ret);
        return ret;
    }
    buffer->frame_info.phy_ptr[2] = static_cast<uint32_t>(phy_ptr2);

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "mmzAlloc index: " << index);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "vio_buf_addr: "   << buffer);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_y_paddr: "    << buffer->frame_info.phy_ptr[0]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_y_vaddr "
         << reinterpret_cast<void*>(buffer->frame_info.vir_ptr[0]));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_u_paddr: "   << buffer->frame_info.phy_ptr[1]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_u_vaddr: "
         << reinterpret_cast<void*>(buffer->frame_info.vir_ptr[1]));
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_v_paddr: "   << buffer->frame_info.phy_ptr[2]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_v_vaddr: "
         << reinterpret_cast<void*>(buffer->frame_info.vir_ptr[2]));

    return 0;
}

int MediaCodecManager::AllocVbBufLane(int index, void *buf, int size,
    int vb_cache_en) {
    int ret;
    iot_venc_src_buf_t *buffer = reinterpret_cast<iot_venc_src_buf_t*>(buf);
    uint64_t phy_ptr;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "enter " << __FUNCTION__);
    if (buffer == nullptr) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "buffer is nullptr error!");
        return -1;
    }

    if (vb_cache_en == 1) {
      ret = HB_SYS_AllocCached(&phy_ptr,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[0]), size);
    } else {
      ret = HB_SYS_Alloc(&phy_ptr,
          reinterpret_cast<void**>(&buffer->frame_info.vir_ptr[0]), size);
    }
    if (ret) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "index: " << index << "alloc vb buffer error, ret: " << ret);
        return ret;
    }
    buffer->frame_info.phy_ptr[0] = static_cast<uint32_t>(phy_ptr);

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "mmzAlloc index: " << index);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "vio_buf_addr: "   << buffer);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_paddr: "    << buffer->frame_info.phy_ptr[0]);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_vaddr "
         << reinterpret_cast<void*>(buffer->frame_info.vir_ptr[0]));

    return 0;
}

int MediaCodecManager::VbBufInit(int chn, int width, int height, int stride,
        int size, int vb_num, int vb_cache_en) {
    std::lock_guard<std::mutex> lg(mutex_);
    int ret;
    uint32_t size_y, size_uv, size_u, size_v;
    iot_venc_src_buf_t *src_buf;
    iot_venc_stream_buf_t *stream_buf;

    vb_num_[chn] = vb_num;
    vb_cache_en_[chn] = vb_cache_en;
    size_y = stride * height;
    size_uv = size - size_y;
    if (vb_num > 0) {
      use_vb_[chn] = 1;
    }

    VP_CONFIG_S vp_config;
    memset(&vp_config, 0x00, sizeof(VP_CONFIG_S));
    vp_config.u32MaxPoolCnt = MAX_POOL_CNT;
    ret = HB_VP_SetConfig(&vp_config);
    if (ret) {
        if (ret == -HB_ERR_VP_BUSY) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "hb vp have already config, ret: " << ret);
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "hb vp config failed, need return, ret: " << ret);
            return ret;
        }
    }

    ret = HB_VP_Init();
    if (ret) {
        if (ret == -HB_ERR_VP_BUSY) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "hb vp have already init, ret: " << ret);
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "hb vp init failed, need return, ret: " << ret);
            return ret;
        }
    }

    PIXEL_FORMAT_E pix_fmt = venc_chn_attr_[chn].stVencAttr.enPixelFormat;

    for (int i = 0; i < vb_num; i++) {
        src_buf = reinterpret_cast<iot_venc_src_buf_t *>\
                             (malloc(sizeof(iot_venc_src_buf_t)));
        // HOBOT_CHECK(src_buf != nullptr) << "malloc src buf failed";
        stream_buf = reinterpret_cast<iot_venc_stream_buf_t *>\
                    (malloc(sizeof(iot_venc_stream_buf_t)));
        // HOBOT_CHECK(stream_buf != nullptr) << "malloc stream buf failed";
        memset(src_buf, 0x00, sizeof(iot_venc_src_buf_t));
        memset(stream_buf, 0x00, sizeof(iot_venc_stream_buf_t));
        if (pix_fmt == HB_PIXEL_FORMAT_NV12 ||
                pix_fmt == HB_PIXEL_FORMAT_NV21 ||
                pix_fmt == HB_PIXEL_FORMAT_NV16 ||
                pix_fmt == HB_PIXEL_FORMAT_NV61) {
            ret = AllocVbBuf2Lane(i, src_buf, size_y, size_uv, vb_cache_en);
            if (ret) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "alloc vb buffer 2 land failed, ret: " << ret);
                return ret;
            }
        } else if (pix_fmt == HB_PIXEL_FORMAT_YUYV422 ||
                pix_fmt == HB_PIXEL_FORMAT_YVYU422 ||
                pix_fmt == HB_PIXEL_FORMAT_UYVY422 ||
                pix_fmt == HB_PIXEL_FORMAT_VYUY422 ||
                pix_fmt == HB_PIXEL_FORMAT_YUV400) {
            ret = AllocVbBufLane(i, src_buf, size, vb_cache_en);
            if (ret) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "alloc vb buffer 1 land failed, ret: " << ret);
                return ret;
            }
        } else if (pix_fmt == HB_PIXEL_FORMAT_YUV422P ||
                pix_fmt == HB_PIXEL_FORMAT_YUV420P) {
            size_u = size_uv / 2;
            size_v = size_uv / 2;
            ret = AllocVbBuf3Lane(i, src_buf, size_y, size_u, size_v,
                vb_cache_en);
            if (ret) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "alloc vb buffer 1 land failed, ret: " << ret);
                return ret;
            }
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "no support JPEG encode format!!!");
            return -1;
        }
        src_buf->frame_info.pix_format = pix_fmt;
        src_buf->frame_info.size = size;
        src_buf->frame_info.width = width;
        src_buf->frame_info.height = height;
        src_buf->frame_info.stride = stride;
        vb_src_buf_queue_[chn].push(src_buf);
        vb_stream_buf_queue_[chn].push(stream_buf);
    }

    return 0;
}

int MediaCodecManager::GetVbBuf(int chn, iot_venc_src_buf_t **buf) {
    std::lock_guard<std::mutex> lg(mutex_);
    iot_venc_src_buf_t *vb_buf;

    if (vb_src_buf_queue_[chn].size() == 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "vb src buf queue size is zero, error!");
        return -1;
    }
    vb_buf = vb_src_buf_queue_[chn].front();
    vb_src_buf_queue_[chn].pop();
    if (vb_buf == nullptr) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "vb buf is nullptr");
        return -1;
    }
    *buf = vb_buf;

    return 0;
}

int MediaCodecManager::FreeVbBuf(int chn, iot_venc_src_buf_t *buf) {
    std::lock_guard<std::mutex> lg(mutex_);
    int ret = 0;

    if (buf == nullptr) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "buffer is nullptr error!");
        ret = -1;
        return ret;
    }

    vb_src_buf_queue_[chn].push(buf);  // push again for cycle queue

    return ret;
}

int MediaCodecManager::VbBufDeInit(int chn) {
    std::lock_guard<std::mutex> lg(mutex_);
    int ret, cnt;
    iot_venc_src_buf_t *buffer;
    iot_venc_stream_buf_t *stream_buffer;

    cnt = vb_src_buf_queue_[chn].size();
    for (int i = 0; i < cnt; i++) {
        buffer = vb_src_buf_queue_[chn].front();
        vb_src_buf_queue_[chn].pop();
        if (buffer == nullptr) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "vb buf is nullptr");
            return -1;
        }
        ret = HB_SYS_Free(buffer->frame_info.phy_ptr[0],
                buffer->frame_info.vir_ptr[0]);
        if (ret == 0) {
            if (buffer->frame_info.vir_ptr[1] != NULL) {
                ret = HB_SYS_Free(buffer->frame_info.phy_ptr[1],
                        buffer->frame_info.vir_ptr[1]);
                if (ret == 0) {
                    if (buffer->frame_info.vir_ptr[2] != NULL) {
                        ret = HB_SYS_Free(buffer->frame_info.phy_ptr[2],
                                buffer->frame_info.vir_ptr[2]);
                        if (ret != 0) {
                            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "hb sys free v vio buf: " << i
                                << "failed, ret: " << ret);
                            return ret;
                        }
                    }
                } else {
                    RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "hb sys free u vio buf: " << i
                         << "failed, ret: " << ret);
                    return ret;
                }
            }
        } else {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "hb sys free y vio buf: " << i << "failed, ret: " << ret);
            return ret;
        }
        free(buffer);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "mmzFree index: "  << i);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "vio_buf_addr: "   << buffer);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_y_paddr: "    << buffer->frame_info.phy_ptr[0]);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_y_vaddr "     << reinterpret_cast<void*>\
            (buffer->frame_info.vir_ptr[0]));
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_u_paddr: "   << buffer->frame_info.phy_ptr[1]);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_u_vaddr: "   << reinterpret_cast<void*>\
            (buffer->frame_info.vir_ptr[1]));
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_v_paddr: "   << buffer->frame_info.phy_ptr[2]);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "buf_v_vaddr: "   << reinterpret_cast<void*>\
            (buffer->frame_info.vir_ptr[2]));
    }

    if (use_vb_[chn]) {
      int stream_buf_cnt = vb_stream_buf_queue_[chn].size();
      for (int i = 0; i < stream_buf_cnt; i++) {
        stream_buffer = vb_stream_buf_queue_[chn].front();
        vb_stream_buf_queue_[chn].pop();
        if (stream_buffer == nullptr) {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "stream buf is nullptr");
          return -1;
        }
        free(stream_buffer);
      }
    }

    ret = HB_VP_Exit();
    if (ret == 0) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("websocket"), "vp exit ok!");
    } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("websocket"), "vp exit error!");
        return ret;
    }

    return 0;
}

}  // namespace vision
}  // namespace horizon
