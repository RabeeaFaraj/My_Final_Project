import requests

class Telegram:

    def __init__(self, token='7418774889:AAHdwROdHNLjqabt6hfl5t3mAqjM0UStgi8', chat_id='1645618303', message='target detected'):
        self.token = token
        self.chat_id = chat_id
        self.message = message
        self.url = f"https://api.telegram.org/bot{self.token}/sendMessage?chat_id={self.chat_id}&text={self.message}"

    def send_message(self):
        response = requests.get(self.url)
        return response.json()



