/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __drake_lcmt_whole_body_data_hpp__
#define __drake_lcmt_whole_body_data_hpp__

#include "lcm/lcm_coretypes.h"

#include <vector>

namespace drake
{

class lcmt_whole_body_data
{
    public:
        /// The timestamp in milliseconds.
        int64_t    timestamp;

        int32_t    num_positions;

        std::vector< double > q_des;

        int32_t    num_constrained_dofs;

        std::vector< int32_t > constrained_dofs;

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
         * Returns "lcmt_whole_body_data"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int lcmt_whole_body_data::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int lcmt_whole_body_data::decode(const void *buf, int offset, int maxlen)
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

int lcmt_whole_body_data::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t lcmt_whole_body_data::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* lcmt_whole_body_data::getTypeName()
{
    return "lcmt_whole_body_data";
}

int lcmt_whole_body_data::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_positions, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_positions > 0) {
        tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->q_des[0], this->num_positions);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_constrained_dofs, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_constrained_dofs > 0) {
        tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->constrained_dofs[0], this->num_constrained_dofs);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int lcmt_whole_body_data::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_positions, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_positions) {
        this->q_des.resize(this->num_positions);
        tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->q_des[0], this->num_positions);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_constrained_dofs, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->num_constrained_dofs) {
        this->constrained_dofs.resize(this->num_constrained_dofs);
        tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->constrained_dofs[0], this->num_constrained_dofs);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int lcmt_whole_body_data::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, this->num_positions);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, this->num_constrained_dofs);
    return enc_size;
}

uint64_t lcmt_whole_body_data::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0x0ec1b6910198d358LL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
