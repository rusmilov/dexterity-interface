export class LLMChat {
    constructor(rosConnection, divID, chatHistoryID) {
      this.ros = rosConnection
      this.divID = divID;
      this.chatHistoryID = chatHistoryID;

      this.llmListener = new ROSLIB.Topic({
        ros : this.ros,
        name : '/llm_response',
        messageType : 'std_msgs/String'
      });

      this.llmListener.subscribe((message) => this.llmListenerCallback(message));

      this.userResponsePub = new ROSLIB.Topic({
        ros : this.ros,
        name : '/user_response',
        messageType : 'std_msgs/String'
      });

    }

    llmListenerCallback(message) {
      // console.log('Received message on ' + this.llmListener.name + ': ' + message.data);
      let historyElem = document.getElementById(this.chatHistoryID);
      const text = document.createElement("pre");
      text.textContent = `AGENT:\n${message.data}`;
      historyElem.appendChild(text);
      this.llmListener.unsubscribe();

    }
    

    userInputCallback(inputID) {
        let elem = document.getElementById(inputID);
        let value = elem.value;
        
        let str = new ROSLIB.Message({data : value});
        this.userResponsePub.publish(str);

        let historyElem = document.getElementById(this.chatHistoryID);
        const text = document.createElement("div");
        text.textContent = `USER: ${value}`;
        historyElem.appendChild(text);

        console.log(str)
    }

  }
