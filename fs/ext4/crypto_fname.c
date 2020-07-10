/*
 * linux/fs/ext4/crypto_fname.c
 *
 * Copyright (C) 2015, Google, Inc.
 *
 * This contains functions for filename crypto management in ext4
 *
 * Written by Uday Savagaonkar, 2014.
 *
 * This has not yet undergone a rigorous security audit.
 *
 */

#include <crypto/hash.h>
#include <crypto/sha.h>
#include <keys/encrypted-type.h>
#include <keys/user-type.h>
#include <linux/crypto.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/key.h>
#include <linux/list.h>
#include <linux/mempool.h>
#include <linux/random.h>
#include <linux/scatterlist.h>
#include <linux/spinlock_types.h>

#include "ext4.h"
#include "ext4_crypto.h"
#include "xattr.h"

/**
 * ext4_dir_crypt_complete() -
 */
static void ext4_dir_crypt_complete(struct crypto_async_request *req, int res)
{
	struct ext4_completion_result *ecr = req->data;

	if (res == -EINPROGRESS)
		return;
	ecr->res = res;
	complete(&ecr->completion);
}

bool ext4_valid_filenames_enc_mode(uint32_t mode)
{
	return (mode == EXT4_ENCRYPTION_MODE_AES_256_CTS);
}

static unsigned max_name_len(struct inode *inode)
{
	return S_ISLNK(inode->i_mode) ? inode->i_sb->s_blocksize :
		EXT4_NAME_LEN;
}

/**
 * ext4_fname_encrypt() -
 *
 * This function encrypts the input filename, and returns the length of the
 * ciphertext. Errors are returned as negative numbers.  We trust the caller to
 * allocate sufficient memory to oname string.
 */
static int ext4_fname_encrypt(struct inode *inode,
			      const struct qstr *iname,
			      struct ext4_str *oname)
{
	u32 ciphertext_len;
	struct ablkcipher_request *req = NULL;
	DECLARE_EXT4_COMPLETION_RESULT(ecr);
	struct ext4_crypt_info *ci = EXT4_I(inode)->i_crypt_info;
	struct crypto_ablkcipher *tfm = ci->ci_ctfm;
	int res = 0;
	char iv[EXT4_CRYPTO_BLOCK_SIZE];
	struct scatterlist src_sg, dst_sg;
	int padding = 4 << (ci->ci_flags & EXT4_POLICY_FLAGS_PAD_MASK);
	char *workbuf, buf[32], *alloc_buf = NULL;
	unsigned lim = max_name_len(inode);

	if (iname->len <= 0 || iname->len > lim)
		return -EIO;

	ciphertext_len = (iname->len < EXT4_CRYPTO_BLOCK_SIZE) ?
		EXT4_CRYPTO_BLOCK_SIZE : iname->len;
	ciphertext_len = ext4_fname_crypto_round_up(ciphertext_len, padding);
	ciphertext_len = (ciphertext_len > lim)
			? lim : ciphertext_len;

	if (ciphertext_len <= sizeof(buf)) {
		workbuf = buf;
	} else {
		alloc_buf = kmalloc(ciphertext_len, GFP_NOFS);
		if (!alloc_buf)
			return -ENOMEM;
		workbuf = alloc_buf;
	}

	/* Allocate request */
	req = ablkcipher_request_alloc(tfm, GFP_NOFS);
	if (!req) {
		printk_ratelimited(
		    KERN_ERR "%s: crypto_request_alloc() failed\n", __func__);
		kfree(alloc_buf);
		return -ENOMEM;
	}
	ablkcipher_request_set_callback(req,
		CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
		ext4_dir_crypt_complete, &ecr);

	/* Copy the input */
	memcpy(workbuf, iname->name, iname->len);
	if (iname->len < ciphertext_len)
		memset(workbuf + iname->len, 0, ciphertext_len - iname->len);

	/* Initialize IV */
	memset(iv, 0, EXT4_CRYPTO_BLOCK_SIZE);

	/* Create encryption request */
	sg_init_one(&src_sg, workbuf, ciphertext_len);
	sg_init_one(&dst_sg, oname->name, ciphertext_len);
	ablkcipher_request_set_crypt(req, &src_sg, &dst_sg, ciphertext_len, iv);
	res = crypto_ablkcipher_encrypt(req);
	if (res == -EINPROGRESS || res == -EBUSY) {
		wait_for_completion(&ecr.completion);
		res = ecr.res;
	}
	kfree(alloc_buf);
	ablkcipher_request_free(req);
	if (res < 0) {
		printk_ratelimited(
		    KERN_ERR "%s: Error (error code %d)\n", __func__, res);
	}
	oname->len = ciphertext_len;
	return res;
}

/*
 * ext4_fname_decrypt()
 *	This function decrypts the input filename, and returns
 *	the length of the plaintext.
 *	Errors are returned as negative numbers.
 *	We trust the caller to allocate sufficient memory to oname string.
 */
static int ext4_fname_decrypt(struct inode *inode,
			      const struct ext4_str *iname,
			      struct ext4_str *oname)
{
	struct ext4_str tmp_in[2], tmp_out[1];
	struct ablkcipher_request *req = NULL;
	DECLARE_EXT4_COMPLETION_RESULT(ecr);
	struct scatterlist src_sg, dst_sg;
	struct ext4_crypt_info *ci = EXT4_I(inode)->i_crypt_info;
	struct crypto_ablkcipher *tfm = ci->ci_ctfm;
	int res = 0;
	char iv[EXT4_CRYPTO_BLOCK_SIZE];
	unsigned lim = max_name_len(inode);

	if (iname->len <= 0 || iname->len > lim)
		return -EIO;

	tmp_in[0].name = iname->name;
	tmp_in[0].len = iname->len;
	tmp_out[0].name = oname->name;

	/* Allocate request */
	req = ablkcipher_request_alloc(tfm, GFP_NOFS);
	if (!req) {
		printk_ratelimited(
		    KERN_ERR "%s: crypto_request_alloc() failed\n",  __func__);
		return -ENOMEM;
	}
	ablkcipher_request_set_callback(req,
		CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
		ext4_dir_crypt_complete, &ecr);

	/* Initialize IV */
	memset(iv, 0, EXT4_CRYPTO_BLOCK_SIZE);

	/* Create encryption request */
	sg_init_one(&src_sg, iname->name, iname->len);
	sg_init_one(&dst_sg, oname->name, oname->len);
	ablkcipher_request_set_crypt(req, &src_sg, &dst_sg, iname->len, iv);
	res = crypto_ablkcipher_decrypt(req);
	if (res == -EINPROGRESS || res == -EBUSY) {
		wait_for_completion(&ecr.completion);
		res = ecr.res;
	}
	ablkcipher_request_free(req);
	if (res < 0) {
		printk_ratelimited(
		    KERN_ERR "%s: Error in ext4_fname_encrypt (error code %d)\n",
		    __func__, res);
		return res;
	}

	oname->len = strnlen(oname->name, iname->len);
	return oname->len;
}

/**
 * ext4_fname_encode_digest() -
 *
 * Encodes the input digest using characters from the set [a-zA-Z0-9_+].
 * The encoded string is roughly 4/3 times the size of the input string.
 */
int ext4_fname_encode_digest(char *dst, char *src, u32 len)
{
	static const char *lookup_table =
		"abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_+";
	u32 current_chunk, num_chunks, i;
	char tmp_buf[3];
	u32 c0, c1, c2, c3;

	current_chunk = 0;
	num_chunks = len/3;
	for (i = 0; i < num_chunks; i++) {
		c0 = src[3*i] & 0x3f;
		c1 = (((src[3*i]>>6)&0x3) | ((src[3*i+1] & 0xf)<<2)) & 0x3f;
		c2 = (((src[3*i+1]>>4)&0xf) | ((src[3*i+2] & 0x3)<<4)) & 0x3f;
		c3 = (src[3*i+2]>>2) & 0x3f;
		dst[4*i] = lookup_table[c0];
		dst[4*i+1] = lookup_table[c1];
		dst[4*i+2] = lookup_table[c2];
		dst[4*i+3] = lookup_table[c3];
	}
	if (i*3 < len) {
		memset(tmp_buf, 0, 3);
		memcpy(tmp_buf, &src[3*i], len-3*i);
		c0 = tmp_buf[0] & 0x3f;
		c1 = (((tmp_buf[0]>>6)&0x3) | ((tmp_buf[1] & 0xf)<<2)) & 0x3f;
		c2 = (((tmp_buf[1]>>4)&0xf) | ((tmp_buf[2] & 0x3)<<4)) & 0x3f;
		c3 = (tmp_buf[2]>>2) & 0x3f;
		dst[4*i] = lookup_table[c0];
		dst[4*i+1] = lookup_table[c1];
		dst[4*i+2] = lookup_table[c2];
		dst[4*i+3] = lookup_table[c3];
		i++;
	}
	return (i * 4);
}

/**
 * ext4_fname_hash() -
 *
 * This function computes the hash of the input filename, and sets the output
 * buffer to the *encoded* digest.  It returns the length of the digest as its
 * return value.  Errors are returned as negative numbers.  We trust the caller
 * to allocate sufficient memory to oname string.
 */
static int ext4_fname_hash(struct ext4_fname_crypto_ctx *ctx,
			   const struct ext4_str *iname,
			   struct ext4_str *oname)
{
	struct scatterlist sg;
	struct hash_desc desc = {
		.tfm = (struct crypto_hash *)ctx->htfm,
		.flags = CRYPTO_TFM_REQ_MAY_SLEEP
	};
	int res = 0;

	if (iname->len <= EXT4_FNAME_CRYPTO_DIGEST_SIZE) {
		res = ext4_fname_encode_digest(oname->name, iname->name,
					       iname->len);
		oname->len = res;
		return res;
	}

	sg_init_one(&sg, iname->name, iname->len);
	res = crypto_hash_init(&desc);
	if (res) {
		printk(KERN_ERR
		       "%s: Error initializing crypto hash; res = [%d]\n",
		       __func__, res);
		goto out;
	}
	res = crypto_hash_update(&desc, &sg, iname->len);
	if (res) {
		printk(KERN_ERR
		       "%s: Error updating crypto hash; res = [%d]\n",
		       __func__, res);
		goto out;
	}
	res = crypto_hash_final(&desc,
		&oname->name[EXT4_FNAME_CRYPTO_DIGEST_SIZE]);
	if (res) {
		printk(KERN_ERR
		       "%s: Error finalizing crypto hash; res = [%d]\n",
		       __func__, res);
		goto out;
	}
	/* Encode the digest as a printable string--this will increase the
	 * size of the digest */
	oname->name[0] = 'I';
	res = ext4_fname_encode_digest(oname->name+1,
		&oname->name[EXT4_FNAME_CRYPTO_DIGEST_SIZE],
		EXT4_FNAME_CRYPTO_DIGEST_SIZE) + 1;
	oname->len = res;
out:
	return res;
}

/**
 * ext4_fname_crypto_round_up() -
 *
 * Return: The next multiple of block size
 */
u32 ext4_fname_crypto_round_up(u32 size, u32 blksize)
{
	return ((size+blksize-1)/blksize)*blksize;
}

unsigned ext4_fname_encrypted_size(struct inode *inode, u32 ilen)
{
	struct ext4_crypt_info *ci = EXT4_I(inode)->i_crypt_info;
	int padding = 32;

	if (ci)
		padding = 4 << (ci->ci_flags & EXT4_POLICY_FLAGS_PAD_MASK);
	if (ilen < EXT4_CRYPTO_BLOCK_SIZE)
		ilen = EXT4_CRYPTO_BLOCK_SIZE;
	return ext4_fname_crypto_round_up(ilen, padding);
}

/*
 * ext4_fname_crypto_alloc_buffer() -
 *
 * Allocates an output buffer that is sufficient for the crypto operation
 * specified by the context and the direction.
 */
int ext4_fname_crypto_alloc_buffer(struct inode *inode,
				   u32 ilen, struct ext4_str *crypto_str)
{
	unsigned int olen = ext4_fname_encrypted_size(inode, ilen);

	crypto_str->len = olen;
	if (olen < EXT4_FNAME_CRYPTO_DIGEST_SIZE*2)
		olen = EXT4_FNAME_CRYPTO_DIGEST_SIZE*2;
	/* Allocated buffer can hold one more character to null-terminate the
	 * string */
	crypto_str->name = kmalloc(olen+1, GFP_NOFS);
	if (!(crypto_str->name))
		return -ENOMEM;
	return 0;
}

/**
 * ext4_fname_crypto_free_buffer() -
 *
 * Frees the buffer allocated for crypto operation.
 */
void ext4_fname_crypto_free_buffer(struct ext4_str *crypto_str)
{
	if (!crypto_str)
		return;
	kfree(crypto_str->name);
	crypto_str->name = NULL;
}

/**
 * ext4_fname_disk_to_usr() - converts a filename from disk space to user space
 */
int _ext4_fname_disk_to_usr(struct inode *inode,
			    struct dx_hash_info *hinfo,
			    const struct ext4_str *iname,
			    struct ext4_str *oname)
{
	char buf[24];
	int ret;

	if (iname->len < 3) {
		/*Check for . and .. */
		if (iname->name[0] == '.' && iname->name[iname->len-1] == '.') {
			oname->name[0] = '.';
			oname->name[iname->len-1] = '.';
			oname->len = iname->len;
			return oname->len;
		}
	}
	if (iname->len < EXT4_CRYPTO_BLOCK_SIZE) {
		EXT4_ERROR_INODE(inode, "encrypted inode too small");
		return -EUCLEAN;
	}
	if (EXT4_I(inode)->i_crypt_info)
		return ext4_fname_decrypt(inode, iname, oname);

	if (iname->len <= EXT4_FNAME_CRYPTO_DIGEST_SIZE) {
		ret = digest_encode(iname->name, iname->len, oname->name);
		oname->len = ret;
		return ret;
	}
	if (hinfo) {
		memcpy(buf, &hinfo->hash, 4);
		memcpy(buf+4, &hinfo->minor_hash, 4);
	} else
		memset(buf, 0, 8);
	memcpy(buf + 8, iname->name + iname->len - 16, 16);
	oname->name[0] = '_';
	ret = digest_encode(buf, 24, oname->name+1);
	oname->len = ret + 1;
	return ret + 1;
}

int ext4_fname_disk_to_usr(struct inode *inode,
			   struct dx_hash_info *hinfo,
			   const struct ext4_dir_entry_2 *de,
			   struct ext4_str *oname)
{
	struct ext4_str iname = {.name = (unsigned char *) de->name,
				 .len = de->name_len };

	return _ext4_fname_disk_to_usr(inode, hinfo, &iname, oname);
}


/**
 * ext4_fname_usr_to_disk() - converts a filename from user space to disk space
 */
int ext4_fname_usr_to_disk(struct inode *inode,
			   const struct qstr *iname,
			   struct ext4_str *oname)
{
	int res;
	struct ext4_crypt_info *ci = EXT4_I(inode)->i_crypt_info;

	if (iname->len < 3) {
		/*Check for . and .. */
		if (iname->name[0] == '.' &&
				iname->name[iname->len-1] == '.') {
			oname->name[0] = '.';
			oname->name[iname->len-1] = '.';
			oname->len = iname->len;
			return oname->len;
		}
	}
	if (ci) {
		res = ext4_fname_encrypt(inode, iname, oname);
		return res;
	}
	/* Without a proper key, a user is not allowed to modify the filenames
	 * in a directory. Consequently, a user space name cannot be mapped to
	 * a disk-space name */
	return -EACCES;
}

int ext4_fname_setup_filename(struct inode *dir, const struct qstr *iname,
			      int lookup, struct ext4_filename *fname)
{
	struct ext4_crypt_info *ci;
	int ret = 0, bigname = 0;

	memset(fname, 0, sizeof(struct ext4_filename));
	fname->usr_fname = iname;

	if (!ext4_encrypted_inode(dir) ||
	    ((iname->name[0] == '.') &&
	     ((iname->len == 1) ||
	      ((iname->name[1] == '.') && (iname->len == 2))))) {
		fname->disk_name.name = (unsigned char *) iname->name;
		fname->disk_name.len = iname->len;
		return 0;
	}
	ret = ext4_get_encryption_info(dir);
	if (ret)
		return ret;
	ci = EXT4_I(dir)->i_crypt_info;
	if (ci) {
		ret = ext4_fname_crypto_alloc_buffer(dir, iname->len,
						     &fname->crypto_buf);
		if (ret < 0)
			return ret;
		ret = ext4_fname_encrypt(dir, iname, &fname->crypto_buf);
		if (ret < 0)
			goto errout;
		fname->disk_name.name = fname->crypto_buf.name;
		fname->disk_name.len = fname->crypto_buf.len;
		return 0;
	}
	if (!lookup)
		return -EACCES;

	/* We don't have the key and we are doing a lookup; decode the
	 * user-supplied name
	 */
	if (iname->name[0] == '_')
		bigname = 1;
	if ((bigname && (iname->len != 33)) ||
	    (!bigname && (iname->len > 43)))
		return -ENOENT;

	fname->crypto_buf.name = kmalloc(32, GFP_KERNEL);
	if (fname->crypto_buf.name == NULL)
		return -ENOMEM;
	ret = digest_decode(iname->name + bigname, iname->len - bigname,
			    fname->crypto_buf.name);
	if (ret < 0) {
		ret = -ENOENT;
		goto errout;
	}
	fname->crypto_buf.len = ret;
	if (bigname) {
		memcpy(&fname->hinfo.hash, fname->crypto_buf.name, 4);
		memcpy(&fname->hinfo.minor_hash, fname->crypto_buf.name + 4, 4);
	} else {
		fname->disk_name.name = fname->crypto_buf.name;
		fname->disk_name.len = fname->crypto_buf.len;
	}
	return 0;
errout:
	kfree(fname->crypto_buf.name);
	fname->crypto_buf.name = NULL;
	return ret;
}

void ext4_fname_free_filename(struct ext4_filename *fname)
{
	kfree(fname->crypto_buf.name);
	fname->crypto_buf.name = NULL;
	fname->usr_fname = NULL;
	fname->disk_name.name = NULL;
}
