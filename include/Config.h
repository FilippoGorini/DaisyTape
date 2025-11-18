#pragma once
#ifndef DAISYTAPE_CONFIG_H
#define DAISYTAPE_CONFIG_H

/**
 * @brief This static constant defines the maximum block size for all processors.
 * Set to 256 to provide adequate scratch buffer space for all modules.
 */
#define SAFE_MAX_BLOCK_SIZE 256

/**
 * @brief Legacy constant for daisysp compatibility, derived from the safe block size.
 * Note: kMaxBlockSize is not strictly used internally by TapeProcessor but is kept
 * for compatibility with older daisysp versions if needed.
 */
static constexpr int kMaxBlockSize = SAFE_MAX_BLOCK_SIZE;

#endif // DAISYTAPE_CONFIG_H