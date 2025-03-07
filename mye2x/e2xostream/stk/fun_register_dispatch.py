import os
import sys

import logging

class FunctionRegisterDispatcher:
    """
    Register - Store in a dictionary with key value as api name and function name as value
    Dispatch - Call each function one by one and pass the required args
    """
    def __init__(self):
        self.function_map = {}

    def register(self, key, func):
        self.function_map[key] = func

    def dispatch(self, key,state_key, *args, **kwargs):
        if key in self.function_map:
            return self.function_map[key](state_key,*args, **kwargs)
        else:
            message = f"No function mapped to {key}"
            logging.warning(message)  # Log the message as a warning
            return None

