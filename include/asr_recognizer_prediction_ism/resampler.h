/**

Copyright (c) 2016, Aumann Florian, Heller Florian, Hutmacher Robin, Meißner Pascal, Stöckle Patrick, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef RECOGNIZER_PREDICTION_ISM_RESAMPLER_H
#define RECOGNIZER_PREDICTION_ISM_RESAMPLER_H

#include <ISM/common_type/RecognitionResult.hpp>

/* ----------------- Local includes ------------------  */
#include "asr_recognizer_prediction_ism/shared_recognition_results_manager.h"

namespace recognizer_prediction_ism
{
  //Realizes importance sampling on Recognition Result vector in shared memory according to their confidence.
  class Resampler
  {

  public:

      Resampler(bool equal_results_number,
                unsigned int importance_resampled_size,
                SharedRecognitionResultsManagerPtr shared_recognition_results_ptr);
    
    unsigned int getImportanceResampledSize() const;

    //This is the central method of this class.
    std::vector<ISM::RecognitionResult> drawSamples();
    
    bool recognitionResultsAvailable();

  private:

    const bool EQUAL_RESULTS_NUMBER_;
    const unsigned int IMPORTANCE_RESAMPLED_SIZE_;
    const SharedRecognitionResultsManagerPtr shared_recognition_results_manager_ptr_;

  };

  typedef boost::shared_ptr<Resampler> ResamplerPtr;

}


#endif // RECOGNIZER_PREDICTION_ISM_RESAMPLER_H
