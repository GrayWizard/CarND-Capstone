from styx_msgs.msg import TrafficLight
import tensorflow as tf
import cv2
import numpy as np

# WARN: mobilenet will not work on tensorflow 1.0
MODEL = 'inception_2classes.pb'
N_CLASSES = 2

class TLClassifier(object):
    def __init__(self):
        # load labels
	labels_file = 'light_classification/model/output_labels_{}classes.txt'.format(N_CLASSES)
	self.labels = self.load_labels(labels_file)
	 
	model_file = 'light_classification/model/{}'.format(MODEL)
	# load graph, which is stored in the default session
	self.graph = self.load_graph(model_file)

	if 'mobilenet' in MODEL:
	    self.input_layer = 'input:0'
	else:
	    self.input_layer = 'DecodeJpeg/contents:0'

    def load_labels(self, filename):
        """Read in labels, one label per line."""
        return [line.rstrip() for line in tf.gfile.GFile(filename)]
 
    def load_graph(self, filename):
	"""Unpersists graph from file as default graph."""

	graph = tf.Graph()
    	with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(filename, 'rb') as fid:
                serialized_graph = fid.read()
            	od_graph_def.ParseFromString(serialized_graph)
            	tf.import_graph_def(od_graph_def, name='')
        return graph
	
    def run_graph(self, image_data, labels, input_layer_name, output_layer_name,
              num_top_predictions):

	with tf.Session(graph=self.graph) as sess:
	    # Feed the image_data as input to the graph.
	    #   predictions will contain a two-dimensional array, where one
	    #   dimension represents the input image count, and the other has
	    #   predictions per class
	    softmax_tensor = sess.graph.get_tensor_by_name(output_layer_name)
	    predictions, = sess.run(softmax_tensor, {input_layer_name: image_data})

	    # Sort to show labels in order of confidence
	    top_k = predictions.argsort()[-num_top_predictions:]
	    
	    # for some reason mobilenet predictions are inverted
	    if not 'mobilenet' in MODEL:
	    	top_k = top_k[::-1]

	    #print('>> Predictions:')
	    '''for node_id in top_k:
	        human_string = self.labels[node_id]
	        score = predictions[node_id]
	        print('%s (score = %.5f)' % (human_string, score))'''

	return self.labels[top_k[0]]

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

	if 'mobilenet' in MODEL:
	    image = cv2.resize(image, (224, 224), interpolation = cv2.INTER_CUBIC)
	    image = image.astype(np.float)
  	    image = (image - 128.)/128.
	    img_shape = image.shape
  	    image = image.reshape(1, img_shape[0], img_shape[1], img_shape[2])
	else:
	    image = cv2.resize(image, (400, 300), interpolation = cv2.INTER_CUBIC)
	    image = cv2.imencode('.jpg', image)[1].tostring()
        
	state = self.run_graph(image, self.labels, self.input_layer, 'final_result:0', N_CLASSES)
	
	if state == 'red':
	    return TrafficLight.RED
	elif state == 'yellow':
	    return TrafficLight.YELLOW
	elif state == 'green':
	    return TrafficLight.GREEN
	else:
            return TrafficLight.UNKNOWN

