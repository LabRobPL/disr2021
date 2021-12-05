/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __robotlocomotion_viewer_draw_t_hpp__
#define __robotlocomotion_viewer_draw_t_hpp__

#include "lcm/lcm_coretypes.h"

#include <vector>
#include <string>

namespace robotlocomotion
{

class viewer_draw_t
{
    public:
        /// The timestamp in milliseconds.
        int64_t    timestamp;

        int32_t    num_links;

        std::vector< std::string > link_name;

        std::vector< int32_t > robot_num;

        std::vector< std::vector< float > > position;

        std::vector< std::vector< float > > quaternion;

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
         * Returns "viewer_draw_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int viewer_draw_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int viewer_draw_t::decode(const void *buf, int offset, int maxlen)
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

int viewer_draw_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t viewer_draw_t::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* viewer_draw_t::getTypeName()
{
    return "viewer_draw_t";
}

int viewer_draw_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_links, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->num_links; a0++) {
        char* __cstr = const_cast<char*>(this->link_name[a0].c_str());
        tlen = __string_encode_array(
            buf, offset + pos, maxlen - pos, &__cstr, 1);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->num_links > 0) {
        tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->robot_num[0], this->num_links);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    for (int a0 = 0; a0 < this->num_links; a0++) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->position[a0][0], 3);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    for (int a0 = 0; a0 < this->num_links; a0++) {
        tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->quaternion[a0][0], 4);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int viewer_draw_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_links, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    try {
        this->link_name.resize(this->num_links);
    } catch (...) {
        return -1;
    }
    for (int a0 = 0; a0 < this->num_links; a0++) {
        int32_t __elem_len;
        tlen = __int32_t_decode_array(
            buf, offset + pos, maxlen - pos, &__elem_len, 1);
        if(tlen < 0) return tlen; else pos += tlen;
        if(__elem_len > maxlen - pos) return -1;
        this->link_name[a0].assign(static_cast<const char*>(buf) + offset + pos, __elem_len -  1);
        pos += __elem_len;
    }

    if(this->num_links) {
        this->robot_num.resize(this->num_links);
        tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->robot_num[0], this->num_links);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    try {
        this->position.resize(this->num_links);
    } catch (...) {
        return -1;
    }
    for (int a0 = 0; a0 < this->num_links; a0++) {
        if(3) {
            this->position[a0].resize(3);
            tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->position[a0][0], 3);
            if(tlen < 0) return tlen; else pos += tlen;
        }
    }

    try {
        this->quaternion.resize(this->num_links);
    } catch (...) {
        return -1;
    }
    for (int a0 = 0; a0 < this->num_links; a0++) {
        if(4) {
            this->quaternion[a0].resize(4);
            tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->quaternion[a0][0], 4);
            if(tlen < 0) return tlen; else pos += tlen;
        }
    }

    return pos;
}

int viewer_draw_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->num_links; a0++) {
        enc_size += this->link_name[a0].size() + 4 + 1;
    }
    enc_size += __int32_t_encoded_array_size(NULL, this->num_links);
    enc_size += this->num_links * __float_encoded_array_size(NULL, 3);
    enc_size += this->num_links * __float_encoded_array_size(NULL, 4);
    return enc_size;
}

uint64_t viewer_draw_t::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0x20a785ff2d97a122LL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
