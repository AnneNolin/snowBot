# Iridium SBD GMail Downloader using the GMail API, then uploading processed data
# to a Google Sheet using gspread

# Written by Chris Cosgrove 06/07/2020

# Gratefully cribbed code from Paul Clark (PaulZC) and Adam Garbo (adamgarbo)


# Licence: MIT

# This code logs into your GMail account using the API and checks for new SnowBot SBD
# messages. If a new message is found, the code saves the attachment
# to file, and then moves the message to the SBD folder (to free up your inbox). 
# It further processes the data within the message and uploads it to a Google Sheet.

# You will need to create an SBD folder in GMail if it doesn't already exist.

# The code assumes your messages are being delivered by the Rock7 RockBLOCK gateway
# and that the message subject contains the words "Message" "from RockBLOCK".

# Follow these instructions to create your credentials for the API:
# https://developers.google.com/gmail/api/quickstart/python

# Follow these instructions to create your credentials.tokens for the Sheets API
# https://gspread.readthedocs.io/en/latest/oauth2.html

# If modifying these scopes, delete the file token.pickle.
#SCOPES = ['https://www.googleapis.com/auth/gmail.readonly'] # Read only
SCOPES = ['https://www.googleapis.com/auth/gmail.modify'] # Everything except delete
#SCOPES = ['https://mail.google.com/'] # Full permissions

import base64
import pickle
import os.path
from googleapiclient.discovery import build
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
from time import sleep
import pandas as pd
import gspread
import df2gspread as d2g
from oauth2client.service_account import ServiceAccountCredentials

def strip_char(string):
    word_list = [r'\n', 'UTC', ' ']
    string = ' '.join(['' if idx in word_list else idx for idx in string.split()]) 
    return(string)

def messageToDict(message_path):
    message_dict = {}
    with open(message_path, 'r') as f:
        for i in f:
            if len(i) > 2:
                j = i.split(':',1)
                if len(j) > 1: #This controls for any messages starting the email, e.g. 'This email originates from outsode of OSU'
                    j[1] = strip_char(j[1])
                    if j[0] == 'IMEI':
                        message_dict['IMEI'] = int(j[1])

                    if j[0] == 'MOMSN':
                        message_dict['MOMSN'] = int(j[1])

                    if j[0] == 'Transmit Time':
                        message_dict['Transmit Time'] = j[1][:-1]

                    if j[0] == 'Iridium Latitude':
                        message_dict['Iridium Latitude'] = float(j[1])

                    if j[0] == 'Iridium Longitude':
                        message_dict['Iridium Longitude'] = float(j[1])

                    if j[0] == 'Iridium CEP':
                        message_dict['Iridium CEP'] = float(j[1])

                    if j[0] == 'Iridium Session Status':
                        message_dict['Iridium Session Status'] = float(j[1])

                    if j[0] == 'Data':
                        message_dict['Data'] = j[1]
        
    message_df = pd.DataFrame(message_dict, index= [0])
    
    return message_df

def decode_snowBot_data(sbd_raw):
    import binascii
    import logging
    import struct

    # Display binary data as tuples of hex-values
    sbd_hex = binascii.unhexlify(sbd_raw)

    # Specify SBD message length
    sbd_length = 24  

    # Split into individual messages
    sbd_list = [sbd_hex[i:i+sbd_length] for i in range(0, len(sbd_hex), sbd_length)]

    # Empty list of lists
    sbd_lists = []
    
    # Unpack messages and store to list of lists
    for x in sbd_list:

        # Format characters
        sbd_format = '<hLhhhhhhhhh'

        # Unpack data
        sbd_data = list(struct.unpack(sbd_format,x))

        # Revert variables
        sbd_data[0]             # Node number
        sbd_data[1]             # Unix time
        sbd_data[2] /= 100.0    # Temp Int C
        sbd_data[3] /= 100.0    # Temp Ext C
        sbd_data[4] /= 100.0    # Humid %
        sbd_data[5]             # MxBtx mean mm
        sbd_data[6]             # MxBtx std mm
        sbd_data[7]             # MxBtx max mm
        sbd_data[8]             # MxBtx min mm
        sbd_data[9]             # MxBtx nan mm
        sbd_data[10] /= 1000.0  # Voltage V

        sbd_lists.append(sbd_data)

    return(sbd_lists)

def sampleList_to_df(sampleList, message_df):
    from datetime import datetime
    
    columns = ['Node','Unix Time','Internal Temperature (ºC)', 'External Temperature (ºC)', 
                   'Relative Humidity (%)', 'Mean Distance to Surface (mm)', 'Std. Distance to Surface (mm)',
                   'Max. Distance to Surface (mm)', 'Min. Distance to Surface (mm)', 'Nan. Distance to Surface',
                   'Battery Voltage']

    df = pd.DataFrame(sampleList, columns= columns)
    df['Date Time'] = df['Unix Time'].apply(datetime.utcfromtimestamp)
    df['Date Time'] = df['Date Time'].dt.strftime('%Y-%m-%d %H:%M:%S')
    
    cols = df.columns.tolist()
    cols.insert(2, cols.pop(cols.index('Date Time')))
    df = df.reindex(columns= cols)
    
    df = df.assign(**message_df.iloc[0])
    
    return df

def message_to_df(message_path):
    message_df = messageToDict(message_path)
    sampleList = decode_snowBot_data(message_df.Data[0])
    df = sampleList_to_df(sampleList, message_df)
    return df

def get_credentials():
    """Gets valid user credentials from storage.
    If nothing has been stored, or if the stored credentials are invalid,
    the oauthlib flow is completed to obtain the new credentials.
    Returns:
        Credentials, the obtained credential.
    """

    creds = None
    # The file token.pickle stores the user's access and refresh tokens, and is
    # created automatically when the authorization flow completes for the first
    # time. You will need to delete it if you change the SCOPES.
    if os.path.exists('token.pickle'):
        with open('token.pickle', 'rb') as token:
            creds = pickle.load(token)
    # If there are no (valid) credentials available, let the user log in.
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(
                'credentials.json', SCOPES)
            creds = flow.run_local_server(port=0)
        # Save the credentials for the next run
        with open('token.pickle', 'wb') as token:
            pickle.dump(creds, token)
    return creds

def ListMessagesMatchingQuery(service, user_id, query=''):
    """List all Messages of the user's mailbox matching the query.
    Args:
        service: Authorized Gmail API service instance.
        user_id: User's email address. The special value "me"
        can be used to indicate the authenticated user.
        query: String used to filter messages returned.
        Eg.- 'from:user@some_domain.com' for Messages from a particular sender.
    Returns:
        List of Messages that match the criteria of the query. Note that the
        returned list contains Message IDs, you must use get with the
        appropriate ID to get the details of a Message.
    """
    response = service.users().messages().list(userId=user_id,q=query).execute()
    messages = []
    if 'messages' in response:
        messages.extend(response['messages'])

    while 'nextPageToken' in response:
        page_token = response['nextPageToken']
        response = service.users().messages().list(userId=user_id, q=query,pageToken=page_token).execute()
        messages.extend(response['messages'])

    return messages

def SaveAttachments(service, user_id, msg_id):
    """Get and store attachment from Message with given id.
    Args:
        service: Authorized Gmail API service instance.
        user_id: User's email address. The special value "me"
        can be used to indicate the authenticated user.
        msg_id: ID of Message containing attachment.
    """
    message = service.users().messages().get(userId=user_id, id=msg_id).execute()

    #local_date = datetime.datetime.fromtimestamp(float(message['internalDate'])/1000.)
    #date_str = local_date.strftime("%y-%m-%d_%H-%M-%S_")

    for part in message['payload']['parts']:
        if part['filename']:
            if 'data' in part['body']:
                data=part['body']['data']
            else:
                att_id=part['body']['attachmentId']
                att=service.users().messages().attachments().get(userId=user_id, messageId=msg_id,id=att_id).execute()
                data=att['data']
            file_data = base64.urlsafe_b64decode(data.encode('UTF-8'))
            #path = date_str+part['filename']
            path = "messages/"+part['filename']

            with open(path, 'wb') as f:
                f.write(file_data)
                f.close()

def GetMessageBody(contents):
    """Save the message body.
    Assumes plaintext message body.
    Gratefully plagiarised from:
    https://github.com/rtklibexplorer/GMail_RTKLIB/blob/master/email_utils.py
    """
    for part in contents['payload']['parts']:
        if part['mimeType'] == 'text/plain':
            body = part['body']['data']
            return base64.urlsafe_b64decode(body.encode('UTF-8')).decode('UTF-8')
        elif 'parts' in part:
            # go two levels if necessary
            for sub_part in part['parts']:
                if sub_part['mimeType'] == 'text/plain':
                    body = sub_part['body']['data']
                    return base64.urlsafe_b64decode(body.encode('UTF-8')).decode('UTF-8')

def SaveMessageBody(service, user_id, msg_id):
    """Save the body from Message with given id.
    Args:
        service: Authorized Gmail API service instance.
        user_id: User's email address. The special value "me"
        can be used to indicate the authenticated user.
        msg_id: ID of Message.
    """
    message = service.users().messages().get(userId=user_id, id=msg_id).execute()
    file_data = GetMessageBody(message)

    #local_date = datetime.datetime.fromtimestamp(float(message['internalDate'])/1000.)
    #date_str = local_date.strftime("%y-%m-%d_%H-%M-%S_")
    
    subject = GetSubject(service, user_id, msg_id);
    for c in r' []/\;,><&*:%=+@!#^()|?^': # substitute any invalid characters
        subject = subject.replace(c,'_')
 
    #path = date_str+subject+".txt"
    path = "messages/"+subject+".txt"

    with open(path, 'w') as f:
        f.write(file_data)
        f.close()
        
    message_df = message_to_df(path)

    scope = ['https://spreadsheets.google.com/feeds',
             'https://www.googleapis.com/auth/drive']
    credentials = ServiceAccountCredentials.from_json_keyfile_name('/Users/chris/Desktop/snowBot/code/google_sheets_uploader/snowbot-sbd-ce558ad1e790.json', scope)

    spreadsheetId = '1o3KnlQHNjEaVTOllpM4nOGW0KdMiZ5edCTHrKkvTspI'
    sheetName = 'Sheet1'

    client = gspread.authorize(credentials)
    sh = client.open_by_key(spreadsheetId)
    values = message_df.values.tolist()
    sh.values_append(sheetName, {'valueInputOption': 'USER_ENTERED'}, {'values': values})
    

def GetSubject(service, user_id, msg_id):
    """Returns the subject of the message with given id.
    Args:
        service: Authorized Gmail API service instance.
        user_id: User's email address. The special value "me"
        can be used to indicate the authenticated user.
        msg_id: ID of Message.
    """
    subject = ''
    message = service.users().messages().get(userId=user_id, id=msg_id).execute()
    payload = message["payload"]
    headers = payload["headers"]
    for header in headers:
        if header["name"] == "Subject":
            subject = header["value"]
            break
    return subject

def MarkAsRead(service, user_id, msg_id):
    """Marks the message with given id as read.
    Args:
        service: Authorized Gmail API service instance.
        user_id: User's email address. The special value "me"
        can be used to indicate the authenticated user.
        msg_id: ID of Message.
    """
    service.users().messages().modify(userId=user_id, id=msg_id, body={ 'removeLabelIds': ['UNREAD']}).execute()

def MoveToLabel(service, user_id, msg_id, dest):
    """Changes the labels of the message with given id to 'move' it.
    Args:
        service: Authorized Gmail API service instance.
        user_id: User's email address. The special value "me"
        can be used to indicate the authenticated user.
        msg_id: ID of Message.
        dest: destination label
    """
    # Find Label_ID of destination label
    results = service.users().labels().list(userId=user_id).execute()
    labels = results.get('labels', [])
    for label in labels:
        if label['name'] == dest: dest_id = label['id']

    service.users().messages().modify(userId=user_id, id=msg_id, body={ 'addLabelIds': [dest_id]}).execute()
    service.users().messages().modify(userId=user_id, id=msg_id, body={ 'removeLabelIds': ['INBOX']}).execute()

def main():
    """Creates a Gmail API service object.
    Searches for unread messages, with attachments, with "Message" "from RockBLOCK" in the subject.
    Saves the attachment to disk.
    Marks the message as read.
    Moves it to the SBD folder.
    You will need to create the SBD folder in GMail if it doesn't already exist.
    """
    creds = get_credentials()
    service = build('gmail', 'v1', credentials=creds)

    # Include your RockBLOCK IMEI in the subject search if required
    messages = ListMessagesMatchingQuery(service, 'me', 'subject:(Message \"from RockBLOCK\") is:unread has:attachment')
    if messages:
        for message in messages:
            print('Processing: '+GetSubject(service, 'me', message["id"]))
            SaveMessageBody(service, 'me', message["id"])
            SaveAttachments(service, 'me', message["id"])
            MarkAsRead(service, 'me', message["id"])
            MoveToLabel(service, 'me', message["id"], 'SBD')
    else:
        print('No messages found!')
    
    

# if __name__ == '__main__':
#     print('Iridium Beacon GMail API Downloader for RockBLOCK')
#     print('Press Ctrl-C to quit')
#     try:
#         while True:
#             #print('Checking for messages...')
#             main()
#             for i in range(15):
#                 sleep(1) # Sleep
#     except KeyboardInterrupt:
#         print('Ctrl-C received!')

print('Iridium GMail API Downloader and Sheets API Uploader for RockBLOCK')
main()
