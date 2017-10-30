python prepare_images.py \
--input_dir ../misc/classified-training-images \
--output_dir ~/traffic_lights

../../tensorflow/bazel-bin/tensorflow/examples/image_retraining/retrain \
python retrain.py \
--image_dir ~/traffic_lights \
--how_many_training_steps 4000 \
--intermediate_store_frequency 200 \
--output_graph model/mobilenet_2classes_050_quant.pb \
--output_labels model/output_labels_2classes.txt \
--print_misclassified_test_images \
--architecture 'mobilenet_0.50_224_quantized' \
--random_brightness 5 \
--random_scale 5 \
--flip_left_right true
#--validation_batch_size -1 \