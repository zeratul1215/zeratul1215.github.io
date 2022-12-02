var pics =[
    {
        picSource : "../img/redemption.png",
        introduce : "my game work",
        href : 'https://dazha.itch.io/attackscene?secret=f7h2YvyURZEhs4ddqxm7gE1umBo'
    },
    {
        picSource : "../img/github.png",
        introduce : "my github page",
        href : "https://github.com/zeratul1215"
    },
]

var id = 1;
var picSelector;

function createPics(){
    let picShelf = document.querySelector(picSelector);
    if(picShelf){
        if(pics instanceof Array){
                let pic = pics[id];
                let html = "<a href = "+ pic.href +" target = \"_blank\">\n" +
                    "        <div class=\"pic\">\n" +
                    "            <img src = "+ pic.picSource +">\n" +
                    "        </div>\n" +
                    "        <div class=\"txt\" >\n" +
                    "            <span>"+ pic.introduce +"</span>\n" +
                    "        </div>\n" +
                    "    </a>"
                let container = document.getElementById("pics");
                container.innerHTML = html;
                id++;
                if(id >= pics.length){
                    id = 0;
                }
        }
    }
}

function temp(idSelector){
    picSelector = idSelector;
    setInterval("createPics()","2000");
}


