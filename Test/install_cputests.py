import urllib.request
import os

git_url = 'https://raw.githubusercontent.com/SingleStepTests/ProcessorTests/refs/heads/main/nes6502/v1/'

filename_list = [
    '00.json','01.json','05.json','06.json','08.json','09.json','0a.json','0d.json','0e.json','10.json',
    '11.json','15.json','16.json','18.json','19.json','1d.json','1e.json','20.json','21.json','24.json',
    '25.json','26.json','28.json','29.json','2a.json','2c.json','2d.json','2e.json','30.json','31.json',
    '35.json','36.json','38.json','39.json','3d.json','3e.json','40.json','41.json','45.json','46.json',
    '48.json','49.json','4a.json','4c.json','4d.json','4e.json','50.json','51.json','55.json','56.json',
    '58.json','59.json','5d.json','5e.json','60.json','61.json','65.json','66.json','68.json','69.json',
    '6a.json','6c.json','6d.json','6e.json','70.json','71.json','75.json','76.json','78.json','79.json',
    '7d.json','7e.json','81.json','84.json','85.json','86.json','88.json','8a.json','8c.json','8d.json',
    '8e.json','90.json','91.json','94.json','95.json','96.json','98.json','99.json','9a.json','9d.json',
    'a0.json','a1.json','a2.json','a4.json','a5.json','a6.json','a8.json','a9.json','aa.json','ac.json',
    'ad.json','ae.json','b0.json','b1.json','b4.json','b5.json','b6.json','b8.json','b9.json','ba.json',
    'bc.json','bd.json','be.json','c0.json','c1.json','c4.json','c5.json','c6.json','c8.json','c9.json',
    'ca.json','cc.json','cd.json','ce.json','d0.json','d1.json','d5.json','d6.json','d8.json','d9.json',
    'dd.json','de.json','e0.json','e1.json','e4.json','e5.json','e6.json','e8.json','e9.json','ea.json',
    'ec.json','ed.json','ee.json','f0.json','f1.json','f5.json','f6.json','f8.json','f9.json','fd.json',
    'fe.json'
]

def main():

    file_path = ''
    url = ''

    if not os.path.exists('cpu_test_suit'):
        print('creating test directory......')
        os.makedirs('cpu_test_suit')

    print('downloading all the tests.......')
    for file in filename_list:
        url = git_url + file
        file_path = os.path.join('cpu_test_suit', file)
        print(f'downloading {file} from url \n {url}')
        urllib.request.urlretrieve(url, file_path)

if __name__ == '__main__':
    main()