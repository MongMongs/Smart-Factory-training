## - 서버

using System.Net.Sockets;

using System.Net;

using System.Text;

using System.Collections.Generic;



namespace WinFormsApp37

{

    public partial class Form1 : Form

    {

        private static TcpListener server = null;

        private static List<TcpClient> clients = new List<TcpClient>(); // 클라이언트 리스트 관리



        public Form1()

        {

            InitializeComponent();

        }



        private void Form1_Load(object sender, EventArgs e)

        {

            Thread serverThread = new Thread(StartServer);

            serverThread.IsBackground = true;

            serverThread.Start();

        }



        private void StartServer()

        {

            try

            {

                IPAddress serverIP = IPAddress.Parse("127.0.0.1");

                int port = 13001; // 65535 이하의 숫자만 포트번호로 할당 가능

                server = new TcpListener(serverIP, port);

                server.Start();



                textBox1.AppendText("Echo Server 시작~\r\n");



                while (true)

                {

                    TcpClient client = server.AcceptTcpClient();

                    lock (clients)

                    {

                        clients.Add(client); // 클라이언트를 리스트에 추가

                    }

                    textBox1.AppendText("클라이언트가 연결됨\r\n");



                    Thread clientThread = new Thread(ClientAction);

                    clientThread.IsBackground = true;

                    clientThread.Start(client);

                }

            }

            catch (Exception ex)

            {

                MessageBox.Show(ex.ToString());

            }

            finally

            {

                if (server != null)

                {

                    server.Stop();

                }

            }

        }



        private void ClientAction(object obj)

        {

            TcpClient client = (TcpClient)obj;

            try

            {

                using (NetworkStream stream = client.GetStream())

                {

                    while (true) // 클라이언트의 연결이 끊어질 때까지 루프

                    {

                        byte[] buffer = new byte[2048];

                        int bytesRead = stream.Read(buffer, 0, buffer.Length);



                        if (bytesRead == 0) // 연결이 끊어졌을 때

                            break;



                        string receiveMsg = Encoding.UTF8.GetString(buffer, 0, bytesRead);

                        AppendText("클라이언트에게 받은 메시지 : " + receiveMsg + "\r\n");



                        BroadcastMessage(receiveMsg, client); // 메시지 브로드캐스트

                    }

                }

            }

            catch (Exception ex)

            {

                Console.WriteLine(ex.ToString());

            }

            finally

            {

                lock (clients)

                {

                    clients.Remove(client); // 클라이언트가 연결을 끊었을 때 리스트에서 제거

                }

                client.Close();

            }

        }



        private void BroadcastMessage(string message, TcpClient senderClient)

        {

            



            lock (clients) // 클라이언트 리스트 접근 시 동기화

            {

                foreach (TcpClient client in clients)

                {

                    if (client != senderClient) // 보낸 클라이언트에게는 메시지 전송 안 함

                    {

                        try

                        {

                            

                            byte[] echoMsg = Encoding.UTF8.GetBytes(message);

                            NetworkStream stream = client.GetStream();

                            stream.Write(echoMsg, 0, echoMsg.Length);

                        }

                        catch (Exception ex)

                        {

                            AppendText("클라이언트에게 메시지 전송 중 오류 발생: " + ex.Message + "\r\n");

                        }

                    }

                    if (client == senderClient)

                    {

                        try

                        {

                            string total = "나 : " + message;

                            NetworkStream stream = client.GetStream();

                            byte[] sendCMsg = Encoding.UTF8.GetBytes(total);

                            stream.Write(sendCMsg, 0, sendCMsg.Length);

                        }

                        catch (Exception ex)

                        {

                            AppendText("클라이언트에게 메시지 전송 중 오류 발생: " + ex.Message + "\r\n");

                        }

                    }

                }

            }

        }



        private void AppendText(string text)

        {

            if (this.InvokeRequired)

            {

                this.Invoke(new Action<string>(AppendText), new object[] { text });

            }

            else

            {

                textBox1.AppendText(text + Environment.NewLine);

            }

        }

    }

}


using System.IO;
using System.Net.Sockets;
using System.Text;

namespace WinFormsApp38
{
    public partial class Form1 : Form
    {
        private TcpClient client;
        private NetworkStream stream;
        public Form1()
        {
            InitializeComponent();
            client = new TcpClient("127.0.0.1", 13001); // 초기화 시 클라이언트 연결 생성
            stream = client.GetStream(); // 스트림 생성
            Thread receiveThread = new Thread(ReceiveMessages);
            receiveThread.IsBackground = true;
            receiveThread.Start();
        }
        private void ReceiveMessages()
        {
            try
            {
                while (true)
                {
                    byte[] buffer = new byte[2048];
                    int bytes = stream.Read(buffer, 0, buffer.Length);
                    if (bytes > 0)
                    {
                        string receivedMsg = Encoding.UTF8.GetString(buffer, 0, bytes);
                        AppendText(receivedMsg + "\n");
                    }
                }
            }
            catch (Exception ex)
            {
                AppendText("서버로부터 메시지를 받는 중 오류 발생: " + ex.Message + "\r\n");
            }
        }

        private void AppendText(string text)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<string>(AppendText), new object[] { text });
            }
            else
            {
                textBox2.AppendText(text + Environment.NewLine);
            }
        }
        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            try
            {
                string message = textBox1.Text;
                byte[] buffer = Encoding.UTF8.GetBytes(message);
                stream.Write(buffer, 0, buffer.Length);
                textBox1.Clear();
            }
            catch (Exception ex)
            {
                AppendText("메시지를 보내는 중 오류 발생: " + ex.Message + "\r\n");
            }
        }
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            stream?.Close();
            client?.Close();
        }
    }
}
-클라이언트
using System.IO;
using System.Net.Sockets;
using System.Text;

namespace WinFormsApp38
{
    public partial class Form1 : Form
    {
        private TcpClient client;
        private NetworkStream stream;
        public Form1()
        {
            InitializeComponent();
            client = new TcpClient("127.0.0.1", 13001); // 초기화 시 클라이언트 연결 생성
            stream = client.GetStream(); // 스트림 생성
            Thread receiveThread = new Thread(ReceiveMessages);
            receiveThread.IsBackground = true;
            receiveThread.Start();
        }
        private void ReceiveMessages()
        {
            try
            {
                while (true)
                {
                    byte[] buffer = new byte[2048];
                    int bytes = stream.Read(buffer, 0, buffer.Length);
                    if (bytes > 0)
                    {
                        string receivedMsg = Encoding.UTF8.GetString(buffer, 0, bytes);
                        AppendText(receivedMsg + "\n");
                    }
                }
            }
            catch (Exception ex)
            {
                AppendText("서버로부터 메시지를 받는 중 오류 발생: " + ex.Message + "\r\n");
            }
        }

        private void AppendText(string text)
        {
            if (this.InvokeRequired)
            {
                this.Invoke(new Action<string>(AppendText), new object[] { text });
            }
            else
            {
                textBox2.AppendText(text + Environment.NewLine);
            }
        }
        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            try
            {
                string message = textBox1.Text;
                byte[] buffer = Encoding.UTF8.GetBytes(message);
                stream.Write(buffer, 0, buffer.Length);
                textBox1.Clear();
            }
            catch (Exception ex)
            {
                AppendText("메시지를 보내는 중 오류 발생: " + ex.Message + "\r\n");
            }
        }
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            stream?.Close();
            client?.Close();
        }
    }
}
