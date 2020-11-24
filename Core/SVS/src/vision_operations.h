/**
 * @brief Rotate a visual percept 90 degrees clockwise.
 * 
 * @tparam img_T The `image_base` sub-class to be rotated.
 * @param visual The visual percept to be rotated.
 */
template <typename img_T>
void rotate_90(img_T& visual);

/**
 * @brief Rotate a visual percept 180 degrees clockwise.
 * 
 * @tparam img_T The `image_base` sub-class to be rotated.
 * @param visual The visual percept to be rotated.
 */
template <typename img_T>
void rotate_180(img_T& visual);

/**
 * @brief Rotate a visual percept 270 degrees clockwise.
 * 
 * @tparam img_T The `image_base` sub-class to be rotated.
 * @param visual The visual percept to be rotated.
 */
template <typename img_T>
void rotate_270(img_T& visual);

/**
 * @brief Mirror the image on the horizontal axis.
 * 
 * @tparam img_T The `image_base` sub-class to be mirrored.
 * @param visual The visual percept to be mirrored.
 */
template <typename img_T>
void mirror_horizontal(img_T& visual);

/**
 * @brief Mirror the image on the vertical axis.
 * 
 * @tparam img_T The `image_base` sub-class to be mirrored.
 * @param visual The visual percept to be mirrored.
 */
template <typename img_T>
void mirror_vertical(img_T& visual);