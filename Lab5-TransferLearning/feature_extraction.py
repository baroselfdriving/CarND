import pickle
import tensorflow as tf
import numpy as np
# TODO: import Keras layers you need here
from keras.layers import Input, Flatten, Dense
from keras.models import Model

flags = tf.app.flags
FLAGS = flags.FLAGS

# command line flags
flags.DEFINE_string('training_file', '', "Bottleneck features training file (.p)")
flags.DEFINE_string('validation_file', '', "Bottleneck features validation file (.p)")
flags.DEFINE_integer('epochs', 50, "Number of epochs")
flags.DEFINE_integer('batch_size', 256, "batch size")


def load_bottleneck_data(training_file, validation_file):
    """
    Utility function to load bottleneck features.

    Arguments:
        training_file - String
        validation_file - String
    """
    print("Training file", training_file)
    print("Validation file", validation_file)

    with open(training_file, 'rb') as f:
        train_data = pickle.load(f)
    with open(validation_file, 'rb') as f:
        validation_data = pickle.load(f)

    X_train = train_data['features']
    y_train = train_data['labels']
    X_val = validation_data['features']
    y_val = validation_data['labels']

    return X_train, y_train, X_val, y_val


def main(_):
    # load bottleneck data
    X_train, y_train, X_val, y_val = load_bottleneck_data(FLAGS.training_file, FLAGS.validation_file)

    print(X_train.shape, y_train.shape)
    print(X_val.shape, y_val.shape)

    # TODO: define your model and hyperparams here
    # make sure to adjust the number of classes based on
    # the dataset
    # 10 for cifar10
    # 43 for traffic
    nb_classes = len(np.unique(y_train))

    # model
    input_shape = X_train.shape[1:]
    inp = Input(shape=input_shape)
    x = Flatten()(inp)
    x = Dense(nb_classes, activation='softmax')(x)
    model = Model(inp, x)
    model.compile('adam', 'sparse_categorical_crossentropy', ['accuracy']) # no need to one-hot encode with sparse_categorical_crossentropy
    history = model.fit(X_train, y_train, nb_epoch=FLAGS.epochs, batch_size=FLAGS.batch_size, validation_data=(X_val, y_val), shuffle=True)

    # TODO: train your model here


# parses flags and calls the `main` function above
if __name__ == '__main__':
    tf.app.run()

# Following results were obtained on cifar-10:
#
# vgg
# Epoch 50/50
# 1000/1000 [==============================] - 0s - loss: 0.2314 - acc: 0.9530 - val_loss: 0.8987 - val_acc: 0.7127
#
# resnet
# Epoch 50/50
# 1000/1000 [==============================] - 0s - loss: 0.0829 - acc: 1.0000 - val_loss: 0.7942 - val_acc: 0.7333
#
# inception
# Epoch 50/50
# 1000/1000 [==============================] - 0s - loss: 0.0885 - acc: 1.0000 - val_loss: 1.0445 - val_acc: 0.6563

# Following results were obtained on traffic signs dataset
#
# vgg (python feature_extraction.py --training_file vgg-100/vgg_traffic_100_bottleneck_features_train.p --validation_file vgg-100/vgg_traffic_bottleneck_features_validation.p)
# Epoch 50/50
# 4300/4300 [==============================] - 0s - loss: 0.0865 - acc: 0.9947 - val_loss: 0.4239 - val_acc: 0.8722
#
# resnet
# Epoch 50/50
# 4300/4300 [==============================] - 0s - loss: 0.0327 - acc: 1.0000 - val_loss: 0.6149 - val_acc: 0.8098
#
# inception
# Epoch 50/50
# 4300/4300 [==============================] - 0s - loss: 0.0273 - acc: 1.0000 - val_loss: 0.8380 - val_acc: 0.7528

# 
