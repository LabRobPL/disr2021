/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#ifndef __drake_lcmt_desired_dof_motions_hpp__
#define __drake_lcmt_desired_dof_motions_hpp__

#include "lcm/lcm_coretypes.h"

#include <vector>
#include <string>
#include "drake/lcmt_constrained_values.hpp"

namespace drake
{

class lcmt_desired_dof_motions
{
    public:
        /// The timestamp in microseconds.
        int64_t    timestamp;

        int32_t    num_dof;

        std::vector< std::string > dof_names;

        /// Should be same size as num_dof
        drake::lcmt_constrained_values constrained_accelerations;

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
         * Returns "lcmt_desired_dof_motions"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int lcmt_desired_dof_motions::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int lcmt_desired_dof_motions::decode(const void *buf, int offset, int maxlen)
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

int lcmt_desired_dof_motions::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t lcmt_desired_dof_motions::getHash()
{
    static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
    return hash;
}

const char* lcmt_desired_dof_motions::getTypeName()
{
    return "lcmt_desired_dof_motions";
}

int lcmt_desired_dof_motions::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->num_dof, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->num_dof; a0++) {
        char* __cstr = const_cast<char*>(this->dof_names[a0].c_str());
        tlen = __string_encode_array(
            buf, offset + pos, maxlen - pos, &__cstr, 1);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = this->constrained_accelerations._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int lcmt_desired_dof_motions::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->num_dof, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    try {
        this->dof_names.resize(this->num_dof);
    } catch (...) {
        return -1;
    }
    for (int a0 = 0; a0 < this->num_dof; a0++) {
        int32_t __elem_len;
        tlen = __int32_t_decode_array(
            buf, offset + pos, maxlen - pos, &__elem_len, 1);
        if(tlen < 0) return tlen; else pos += tlen;
        if(__elem_len > maxlen - pos) return -1;
        this->dof_names[a0].assign(static_cast<const char*>(buf) + offset + pos, __elem_len -  1);
        pos += __elem_len;
    }

    tlen = this->constrained_accelerations._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int lcmt_desired_dof_motions::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->num_dof; a0++) {
        enc_size += this->dof_names[a0].size() + 4 + 1;
    }
    enc_size += this->constrained_accelerations._getEncodedSizeNoHash();
    return enc_size;
}

uint64_t lcmt_desired_dof_motions::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == lcmt_desired_dof_motions::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, lcmt_desired_dof_motions::getHash };

    uint64_t hash = 0xf6cbc71b90c98dc5LL +
         drake::lcmt_constrained_values::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
