import os
import sys

import logging

class FunctionRegisterDispatcher:
    def __init__(self):
        self.function_map = {}

    def register(self, key, func):
        self.function_map[key] = func

    def dispatch(self, key,state_key, *args, **kwargs):
        if key in self.function_map:
            #logging.info(f"Dispatching function for key: {key}")
            return self.function_map[key](state_key,*args, **kwargs)
        else:
            message = f"No function mapped to {key}"
            logging.warning(message)  # Log the message as a warning
            return None