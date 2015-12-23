/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, UT Arlington
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of UT Arlington nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * curl_testing.cpp
 *
 *  Created on: May 17, 2015
 *      Author: Sumit Kumar Das
 */

//----------------------------------------------CURL Implementation Testing-----------------------------------------------------------------------
std::string result;

void function_pt(void *ptr, size_t size, size_t nmemb, void *stream){

	result = ((const char *) ptr);

	//std::size_t found = str.find("object_name");
}

int main(void)
{
  CURL *curl;
  CURLcode res;

  curl = curl_easy_init();
  if(curl) {
    curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:5984/object_recognition/adb0cf025e87d1fd3726ef5128017e99");
    /* example.com is redirected, so we tell libcurl to follow redirection */
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

    /* Perform the request, res will get the return code */
    res = curl_easy_perform(curl);

    /* Check for errors */
    if(res != CURLE_OK)
      fprintf(stderr, "curl_easy_perform() failed: %s\n",
              curl_easy_strerror(res));
    /* always cleanup */
    curl_easy_cleanup(curl);
  }
	while(true)
	{

		result="empty";
		printf("%s\n", result.c_str());

		CURL *curl;
		CURLcode res;
		std::string root;
		std::string key;
		std::string address;

		key = "4a287f8f720a33149879976c8b0009ae";
		root = "http://localhost:5984/object_recognition/";
		address = root.append(key);

		curl = curl_easy_init();
		if(curl) {
			curl_easy_setopt(curl, CURLOPT_URL, address.c_str());
			curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, function_pt);
			curl_easy_perform(curl);
			curl_easy_cleanup(curl);
		}


		printf("After curl: %s\n", result.c_str());

		sleep(1.0);

	}






	return 0;
}


