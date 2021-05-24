/*
 * classify.ino
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                  110
#define PRELENGTH                     15
#define THRESHOLD                     0.3

#define EUCLIDEAN_THRESHOLD         0.1
#define LOUDNESS_THRESHOLD          300

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

float pca_vec1[110] = {0.0010857824141708022, -0.0015911203565807108, 0.0032846705900205775, 0.004038819388261783, 0.0008767300960683538, -0.006568736398010668, 0.0018965450085596275, 0.005226136027051376, 0.002468276863911609, 0.013619786812760203, -0.00169016509039246, -0.00232288652978069, -0.0061436725023182245, -0.023022139445661643, -0.030817922852286413, -0.051258730413151446, -0.0323222312927787, -0.03310625563681219, -0.04051095648689349, -0.08925657278219779, -0.09796918825736341, -0.15055821217465962, -0.1329105136068727, -0.1299762385886594, -0.17090792350189102, -0.15620260753838155, -0.18205896106319586, -0.11418565696068625, -0.1232545746496396, -0.07356228635247129, -0.03228700778542085, 0.009549569069698218, 0.07719138848761906, 0.10152359536851632, 0.1561527671427665, 0.18842932399901183, 0.197859382928335, 0.19453343725660105, 0.19210936789286565, 0.1765914553867371, 0.19522756824015558, 0.1834629153940792, 0.17683776141616084, 0.1913341786113789, 0.18709085638153858, 0.17887395984914187, 0.18666708479525765, 0.17165412322309037, 0.16441728291278784, 0.17929029547841036, 0.1519138473095778, 0.15712341532648808, 0.13846590680907087, 0.12047566454624585, 0.09872909593150139, 0.08803457793522405, 0.061698277497094584, 0.045951031549315555, 0.00979828297457177, 0.008366111860549478, -0.046385788200896866, -0.07504876928232283, -0.07280270694113325, -0.03198836871640067, -0.13459344201866588, -0.07077241587313422, -0.08758411642836511, -0.07768664911688121, -0.07634884485957126, -0.08324163893018204, -0.06345115425362062, -0.06446946891957345, -0.0663578014961909, -0.06029223565621726, -0.07593611622222576, -0.0724295367493707, -0.052451805155100784, -0.04536538454303446, -0.07357329487520319, -0.052053802256372045, -0.05239139445449025, -0.044479931620665596, -0.04853718573922218, -0.03442941659419918, -0.032176792584506, -0.04540027848076946, -0.03752649340753401, -0.02957082646017158, -0.04666342022920588, -0.044359702789283355, -0.03559409390892652, -0.031351767021255444, -0.030234479207401483, -0.04735727726197432, -0.04167236227794556, -0.02421595444840663, -0.04154432906677012, -0.03200544543962749, -0.04064082895362087, -0.021548834651897775, -0.028510940660415516, -0.02985481367230436, -0.029883752213589663, -0.032999616791579034, -0.01265699831797447, -0.019818700440062495, -0.0031207156577008175, -0.018900206098225354, -0.00869078752334074, -0.010394026040961221};
float pca_vec2[110] = {0.020230459959306724, 0.032393440359406944, 0.031162227126831157, 0.03183885161218747, 0.0121549021984097, 0.015392190404741046, 0.02303865968848993, 0.022897272541160498, 0.015792671870671925, 0.02843466894941945, 0.01387757755673238, 0.004153694208116774, 0.006985863923666282, 0.0028685956760995585, 0.0021411042047076514, 0.034851802771515075, -0.019148432278808244, -0.04625922432023216, -0.11498972106055019, -0.1491430925731177, -0.11278082764887479, -0.12916247480197174, -0.17301121808896286, -0.16868100790727295, -0.13682223933975135, -0.17287273343273524, -0.19359151591228413, -0.18926180323473793, -0.14139041295190732, -0.23511929226722567, -0.22887422229850907, -0.21123178213354563, -0.16646462514109278, -0.11124786602112087, -0.06523559509565538, -0.06590412387567783, -0.02985932424681171, -0.013806343923172372, -0.029809104057458832, -0.03627733831838511, -0.019202336321522184, -0.04706409091961876, -0.01949618017979867, -0.008713890225236417, -0.037461526252568426, 0.009997261791408675, -0.010549107370100369, 0.00021208681980647507, 0.008301012490414863, 0.03950898609237032, 0.0019915976352958063, 0.0021565296220990085, 0.014838993756734512, -0.04225045179814953, -0.07293778952369742, -0.08425918570717435, -0.114611875260066, -0.16220976281649063, -0.1738330530913924, -0.10803236293901869, -0.11579675958733997, -0.04991323318697854, -0.09429764122659028, -0.04180480591041613, 0.031289915507414084, 0.000766619457406987, -0.019595834404212215, 0.028003205463506358, 0.018767521322283993, -0.05039488540740291, 0.030424831885597928, 0.05267398950976214, 0.013837840914775756, 0.04258393339590005, 0.060722216655410687, 0.06402058942436267, 0.05954503705457992, 0.07526284339995216, 0.08743120964565748, 0.07869895886486619, 0.07215915116058982, 0.10659391173525393, 0.09302303066942594, 0.10033395297373467, 0.10118382396739457, 0.10875753883045615, 0.12154957714614328, 0.11355105272067287, 0.0978135098712936, 0.10752887305631094, 0.11750437858619354, 0.12617843528221728, 0.14169752905239769, 0.13657959208410583, 0.1205290963024467, 0.13010923114882736, 0.12791156249642321, 0.0977544303595024, 0.13233246623972936, 0.12143521070496213, 0.11192034145119373, 0.0908654238195867, 0.09353832001554055, 0.09055311446299681, 0.10063753358004167, 0.11156086501816488, 0.08241340432328277, 0.0911866227920746, 0.07779446269265246, 0.06912348675295167};
float projected_mean_vec[2] = {0.0018565564257895441, -0.044442684215368115};
float centroid1[2] = {-0.02132437532259558, -0.023096219947210922};
float centroid2[2] = {0.006327990934130111, 0.014882432359758032};
float centroid3[2] = {-0.029688768615139943, 0.01669358791430243};
float centroid4[2] = {0.0446851530036054, -0.008479800326849527};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
//float proj3 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(MIC_INPUT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
}

void loop(void) {
  if (re_pointer == SIZE) {
    digitalWrite(RED_LED, LOW);

    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      //proj3 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += result[i] * pca_vec1[i];
          proj2 += result[i] * pca_vec2[i];
          //proj3 += result[i] * pca_vec3[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];
      //proj3 -= projected_mean_vec[2];

      // Classification
      // Use the function 'l2_norm' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;
      // YOUR CODE HERE
      float distance = 0;
      for (int i = 0; i < 4; i++) {
        distance = l2_norm(proj1, proj2, centroids[i]);
        if (distance < best_dist) {
          best_dist = distance;
          best_index = i;
        }
      }


      // Compare 'best_dist' against the 'EUCLIDEAN_THRESHOLD' and print the result
      // If 'best_dist' is less than the 'EUCLIDEAN_THRESHOLD', the recording is a word
      // Otherwise, the recording is noise
      // YOUR CODE HERE
      String words[4] = {"apple","tarjan", "muffin", "glass"];
      if (best_dist < EUCLIDEAN_THRESHOLD) {
        Serial.println(words[best_index]);
      } else {
        Serial.println("The recording is noise");
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }
    else {
      Serial.println("Below LOUDNESS_THRESHOLD.");
    }


    delay(2000);
    re_pointer = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void reset_blinker(void) {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (re_pointer < SIZE) {
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  // Set the timer based on 25MHz clock
  TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
  TA2CCTL0 = CCIE;
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
}
