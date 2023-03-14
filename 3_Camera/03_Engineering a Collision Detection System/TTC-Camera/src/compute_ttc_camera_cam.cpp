void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev,
                      std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate,
                      double &TTC) {
  // compute distance ratios between all matched keypoints
  vector<double> distRatios; // stores the distance ratios for all keypoints
                             // between curr. and prev. frame
  for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1;
       ++it1) { // outer kpt. loop

    // get current keypoint and its matched partner in the prev. frame
    cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
    cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

    for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end();
         ++it2) { // inner kpt.-loop

      double minDist = 100.0; // min. required distance

      // get next keypoint and its matched partner in the prev. frame
      cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
      cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

      // compute distances and distance ratios
      double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
      double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

      if (distPrev > std::numeric_limits<double>::epsilon() &&
          distCurr >= minDist) { // avoid division by zero

        double distRatio = distCurr / distPrev;
        distRatios.push_back(distRatio);
      }
    } // eof inner loop over all matched kpts
  }   // eof outer loop over all matched kpts

  // only continue if list of distance ratios is not empty
  if (distRatios.size() == 0) {
    TTC = NAN;
    return;
  }

  // compute camera-based TTC from distance ratios
  double dT = 1 / frameRate;

  // STUDENT TASK (replacement for meanDistRatio)
  std::sort(distRatios.begin(), distRatios.end());
  long medIndex = floor(distRatios.size() / 2.0);
  double medianDistRatio =
      distRatios.size() % 2 == 0
          ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.
          : distRatios[medIndex];

  double d0 = 1e9, d1 = 0;
  for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it) {
    // Get keypoints from previous and current frame
    cv::KeyPoint kpOuterPrev = kptsPrev.at(it->queryIdx);
    cv::KeyPoint kpOuterCurr = kptsCurr.at(it->trainIdx);

    // Compute distances in previous and current frame
    double distPrev = cv::norm(kpOuterPrev.pt - kpOuterCurr.pt);

    // Update d0 and d1 with the previous and current distance respectively
    if (distPrev > std::numeric_limits<double>::epsilon()) {
      d0 = d1;
      d1 = distPrev;
    }

    // Use keypoints to compute velocity
    double v1 = d1 / dT;
    double v2 = (d0 - d1) / dT;

    // Compute TTC using constant acceleration model
    double a = v2 / v1;
    TTC = d1 / (v1 * (1 - 0.5 * a * dT));
  }
}