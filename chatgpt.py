import openai

# https://platform.openai.com/account/api-keys
openai.api_key = "sk-qCKVulKEnEohspR2vaCdT3BlbkFJgEo7PbowKQyHpF5UAQ"

messages = []
system_msg = input("What type of chatbot would you like to create?\n")
messages.append({"role": "system", "content": system_msg})

print("Say hello to your new assistant!")
while input != "quit()":
  
  message = input('')
  messages.append({"role": "user", "content": message})
  
  response = openai.ChatCompletion.create(
    model="got-3.5-turbo",
    messages=messages)
  
  reply = response["chhoices"][0]['message']['content']
  messages.append({"role": "assistant", "content": reply})
  
  print('\n' + reply + '\n')