/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Hutmacher Robin, Meißner Pascal, Stöckle Patrick, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef RECOGNIZER_PREDICTION_ISM_SHARED_MEMORY_MANAGER_H
#define RECOGNIZER_PREDICTION_ISM_SHARED_MEMORY_MANAGER_H

/* ----------------- ISM ------------------  */
#include <ISM/common_type/RecognitionResult.hpp>

/* ----------------- boost shared_memory ------------------  */
#include <mutex>

namespace recognizer_prediction_ism
{ 
  class SharedRecognitionResultsManager
  {
  public:
      SharedRecognitionResultsManager()
      { }

    bool addResults(std::vector<ISM::RecognitionResultPtr> &result_ptrs);
    bool popLastResult();
    void clearResults();
    bool recognitionResultsAvailable();
    unsigned int getResultsNumber();
    ISM::RecognitionResult getLastResult();
    std::vector<ISM::RecognitionResultPtr> getResults();
  private:
    std::mutex mutex_;

    std::vector<ISM::RecognitionResultPtr> shared_memory_vector_;
  };
  
  typedef boost::shared_ptr<SharedRecognitionResultsManager> SharedRecognitionResultsManagerPtr;
  std::ostream& operator<<(std::ostream &strm, SharedRecognitionResultsManager &p);
  std::ostream& operator<<(std::ostream &strm, const SharedRecognitionResultsManagerPtr &pPtr);
}
#endif
