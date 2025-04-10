import requests

class Telegram:

    def __init__(self, token='', chat_id='', message='target detected'):
        self.token = token
        self.chat_id = chat_id
        self.message = message
        self.url = f"https://api.telegram.org/bot{self.token}/sendMessage?chat_id={self.chat_id}&text={self.message}"

    def send_message(self):
        response = requests.get(self.url)
        return response.json()



