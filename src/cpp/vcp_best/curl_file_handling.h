#ifndef __VCP_BEST_CURL_FILE_HANDLING_H__
#define __VCP_BEST_CURL_FILE_HANDLING_H__

// Convenience functions for CURL, see https://curl.haxx.se/libcurl/c/fopen.html

#include <curl/curl.h>

// Type of handle;
enum fcurl_type_e
{
  CFTYPE_NONE = 0,
  CFTYPE_FILE = 1,
  CFTYPE_CURL = 2
};

struct fcurl_data
{
  enum fcurl_type_e type;     /* type of handle */
  union
  {
    CURL *curl;
    FILE *file;
  } handle;                   /* handle */

  char *buffer;               /* buffer to store cached data*/
  size_t buffer_len;          /* currently allocated buffers length */
  size_t buffer_pos;          /* end of data in buffer*/
  int still_running;          /* Is background url fetch still in progress */
};
typedef struct fcurl_data URL_FILE;

/* exported functions */
URL_FILE *url_fopen(CURLM **multi_handle, const char *url, const char *operation, long timeout_ms=1500);
int url_fclose(CURLM **multi_handle, URL_FILE *file);
int url_feof(URL_FILE *file);
size_t url_fread(CURLM *multi_handle, void *ptr, size_t size, size_t nmemb, URL_FILE *file);
char * url_fgets(CURLM *multi_handle, char *ptr, size_t size, URL_FILE *file);

// simple http get
// returns 0 if ok, otherwise CURL error code (or -1 if cannot init)
// specify timeout in seconds (or wait forever, if set to 0)
// request HEAD response if head_only != 0
int url_http_get(const char *url, long timeout = 0L, int head_only = 0);

// check if URL exists
// returns 0 if ok, -1 if cannot init, otherwise CURL error code
// specify timeout in seconds (or wait "forever" (i.e. curl default), if timeout = 0)
int url_available(const char *url, long timeout = 0L);

#endif // __VCP_BEST_CURL_FILE_HANDLING_H__
