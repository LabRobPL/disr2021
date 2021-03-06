/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __drake_lcmt_body_motion_data_hpp__
#define __drake_lcmt_body_motion_data_hpp__

#include "lcm/lcm_coretypes.h"

#include <string>
#include "drake/lcmt_piecewise_polynomial.hpp"

namespace drake
{

class lcmt_body_motion_data
{
    public:
        /// The timestamp in milliseconds.
        int64_t    timestamp;

        std::string body_or_frame_name;

        drake::lcmt_piecewise_polynomial spline;

        double     quat_task_to_world[4];

        double     translation_task_to_world[3];

        double     xyz_kp_multiplier[3];

        double     xyz_damping_ratio_multiplier[3];

        double     expmap_kp_multiplier;

        double     expmap_damping_ratio_multiplier;

        double     weight_multiplier[6];

        int8_t     in_floating_base_nullspace;

        int8_t     control_pose_when_in_contact;

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to read while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "lcmt_body_motion_data"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int lcmt_body_motion_data::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int lcmt_body_motion_data::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int lcmt_body_motion_data::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t lcmt_body_motion_data::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* lcmt_body_motion_data::getTypeName()
{
    return "lcmt_body_motion_data";
}

int lcmt_body_motion_data::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    char* body_or_frame_name_cstr = const_cast<char*>(this->body_or_frame_name.c_str());
    tlen = __string_encode_array(
        buf, offset + pos, maxlen - pos, &body_or_frame_name_cstr, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->spline._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->quat_task_to_world[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->translation_task_to_world[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->xyz_kp_multiplier[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->xyz_damping_ratio_multiplier[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->expmap_kp_multiplier, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->expmap_damping_ratio_multiplier, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->weight_multiplier[0], 6);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->in_floating_base_nullspace, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_encode_array(buf, offset + pos, maxlen - pos, &this->control_pose_when_in_contact, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int lcmt_body_motion_data::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    int32_t __body_or_frame_name_len__;
    tlen = __int32_t_decode_array(
        buf, offset + pos, maxlen - pos, &__body_or_frame_name_len__, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    if(__body_or_frame_name_len__ > maxlen - pos) return -1;
    this->body_or_frame_name.assign(
        static_cast<const char*>(buf) + offset + pos, __body_or_frame_name_len__ - 1);
    pos += __body_or_frame_name_len__;

    tlen = this->spline._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->quat_task_to_world[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->translation_task_to_world[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->xyz_kp_multiplier[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->xyz_damping_ratio_multiplier[0], 3);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->expmap_kp_multiplier, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->expmap_damping_ratio_multiplier, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->weight_multiplier[0], 6);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->in_floating_base_nullspace, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __boolean_decode_array(buf, offset + pos, maxlen - pos, &this->control_pose_when_in_contact, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int lcmt_body_motion_data::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += this->body_or_frame_name.size() + 4 + 1;
    enc_size += this->spline._getEncodedSizeNoHash();
    enc_size += __double_encoded_array_size(NULL, 4);
    enc_size += __double_encoded_array_size(NULL, 3);
    enc_size += __double_encoded_array_size(NULL, 3);
    enc_size += __double_encoded_array_size(NULL, 3);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 6);
    enc_size += __boolean_encoded_array_size(NULL, 1);
    enc_size += __boolean_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t lcmt_body_motion_data::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == lcmt_body_motion_data::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, lcmt_body_motion_data::getHash };

    uint64_t hash = 0x0b033435a268e26fLL +
         drake::lcmt_piecewise_polynomial::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
